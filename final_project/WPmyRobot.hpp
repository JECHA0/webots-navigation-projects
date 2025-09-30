#include "OAmyRobot.hpp"
#include <webots/GPS.hpp>
#include <webots/Compass.hpp>
#include <webots/Lidar.hpp>
#include <webots/Receiver.hpp>
#include <vector>
#include <cstring>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <algorithm>

#define MAX_SPEED 6.28
#define TIME_STEP 64
#define PN_GAIN 50.0
#define CLOS_GAIN 2.0
#define HEADING_GAIN 0.2
#define SWITCH_RADIUS 0.1
#define FLYBY_RADIUS 0.02
#define BASE_SPEED 1
#define VFH_THRESHOLD 2.0

using namespace std;

class WPmyRobot : public OAmyRobot {
public:
    enum NavState { N0, N1, N2, N3, N4 };
    NavState navState = N0;
    GPS *gps;
    Compass *compass;
    Lidar *lidar;
    Receiver *receiver;

    const double *gpsLoc;
    const double *north;
    const float *lidarRangeImage;

    double heading;
    double **waypoints;
    int numWP;
    int currentWP = 0;
    double prevLamT = 0.0;
    double prevDLamT = 0.0;
    double VFHDensity[32];
    double VFHBearing[32];
    double valleyPos[16];
    double valleyWid[16];
    unsigned char numValley;

    vector<bool> waypointFlags;

public:
    WPmyRobot() {
        gps = getGPS("gps");
        gps->enable(TIME_STEP);
        compass = getCompass("compass");
        compass->enable(TIME_STEP);
        lidar = getLidar("lidar");
        lidar->enable(TIME_STEP);
        lidar->enablePointCloud();

        receiver = getReceiver("receiver");
        receiver->enable(TIME_STEP);

        gpsLoc = nullptr;
        north = nullptr;
        lidarRangeImage = nullptr;
        currentWP = 0;
        navState = N0;
        prevLamT = 0.0;
        prevDLamT = 0.0;
        numValley = 0;

        waypoints = nullptr;
        numWP = 0;

        for (int k = 0; k < 32; k++) {
            VFHBearing[k] = 1.275 - k * (2.55 / static_cast<double>(32));
        }
    }

    ~WPmyRobot() {
        if (waypoints) {
            for (int i = 0; i < numWP; i++) {
                delete[] waypoints[i];
            }
            delete[] waypoints;
        }
    }

    double calDistance(double x1, double y1, double x2, double y2) {
        return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
    }

    int findNearestWP(double currentX, double currentY) {
        if (numWP == 0) return -1;
        int minIndex = 0;
        double minDist = calDistance(currentX, currentY, waypoints[0][0], waypoints[0][1]);
        for (int i = 1; i < numWP; i++) {
            double dist = calDistance(currentX, currentY, waypoints[i][0], waypoints[i][1]);
            if (dist < minDist) {
                minDist = dist;
                minIndex = i;
            }
        }
        return minIndex;
    }

    void receiveWP() {
        int queueLength = receiver->getQueueLength();
        if (queueLength == 0) return;

        while (queueLength > 1) {
            receiver->nextPacket();
            queueLength--;
        }

        const void *packet = receiver->getData();
        size_t packetSize = receiver->getDataSize();
        unsigned char buffer[129];
        if (packetSize > sizeof(buffer)) {
            receiver->nextPacket();
            return;
        }
        memcpy(buffer, packet, packetSize);

        int newNumWP = static_cast<int>(buffer[0]);
        size_t expectedSize = 1 + 2 * newNumWP * sizeof(float);
        if (packetSize != expectedSize) {
            receiver->nextPacket();
            return;
        }

        const float *wpData = reinterpret_cast<const float *>(buffer + 1);
        if (waypoints) {
            for (int i = 0; i < numWP; i++) {
                delete[] waypoints[i];
            }
            delete[] waypoints;
        }
        numWP = newNumWP;
        waypoints = new double*[numWP];
        waypointFlags.assign(numWP, false);
        for (int i = 0; i < numWP; i++) {
            waypoints[i] = new double[2];
            waypoints[i][0] = static_cast<double>(wpData[i * 2]);
            waypoints[i][1] = static_cast<double>(wpData[i * 2 + 1]);
        }

        gpsLoc = gps->getValues();
        if (gpsLoc) {
            currentWP = findNearestWP(gpsLoc[0], gpsLoc[1]);
        } else {
            currentWP = 0;
        }

        receiver->nextPacket();
    }

    double normalizeAngle(double angle) {
        while (angle > M_PI) {
            angle -= 2 * M_PI;
        }
        while (angle < -M_PI) {
            angle += 2 * M_PI;
        }
        return angle;
    }

    void setMotorSpeeds(double psi_c) {
        double deltaV = -7 * psi_c;
        double leftSpeed = BASE_SPEED * MAX_SPEED + deltaV;
        double rightSpeed = BASE_SPEED * MAX_SPEED - deltaV;

        leftSpeed = max(-MAX_SPEED, min(MAX_SPEED, leftSpeed));
        rightSpeed = max(-MAX_SPEED, min(MAX_SPEED, rightSpeed));

        leftMotor->setVelocity(leftSpeed);
        rightMotor->setVelocity(rightSpeed);
    }

    void getHeading() {
        north = compass->getValues();
        if (north[0] == 0.0 && north[1] == 0.0) {
            heading = 0.0;
        } else {
            heading = normalizeAngle(atan2(north[0], north[1]));
        }
    }

    double getDistanceToTarget() {
        if (!waypoints || numWP == 0) return INFINITY;
        double Xo = gpsLoc[0];
        double Yo = gpsLoc[1];
        double Xt = waypoints[currentWP][0];
        double Yt = waypoints[currentWP][1];
        return sqrt(pow(Xt - Xo, 2) + pow(Yt - Yo, 2));
    }

    void getTargetBodyCoords(double &xT, double &yT) {
        if (!waypoints || numWP == 0) {
            xT = 0.0;
            yT = 0.0;
            return;
        }
        double Xo = gpsLoc[0];
        double Yo = gpsLoc[1];
        double Xt = waypoints[currentWP][0];
        double Yt = waypoints[currentWP][1];
        xT = cos(heading) * (Xt - Xo) + sin(heading) * (Yt - Yo);
        yT = -sin(heading) * (Xt - Xo) + cos(heading) * (Yt - Yo);
    }

    double getLamT() {
        double xT, yT;
        getTargetBodyCoords(xT, yT);
        if (xT == 0.0 && yT == 0.0) return 0.0;
        return normalizeAngle(atan2(yT, xT));
    }

    double getDLamT() {
        double lamT = getLamT();
        double deltaLamT = normalizeAngle(lamT - prevLamT);
        double dLamT = deltaLamT / TIME_STEP;
        dLamT = 0.7 * dLamT + 0.3 * prevDLamT;
        prevLamT = lamT;
        prevDLamT = dLamT;
        return dLamT;
    }

    double PN_control() {
        double dLamT = getDLamT();
        double lamT = getLamT();
        double headingError = normalizeAngle(lamT);
        return PN_GAIN * dLamT + HEADING_GAIN * headingError;
    }

    double CLOS_control() {
        double lamT = getLamT();
        return CLOS_GAIN * lamT;
    }

    void getVFH() {
        lidarRangeImage = lidar->getRangeImage();
        double r_T = getDistanceToTarget();
        double r_max = min(r_T, 2.0);
        double a = 2.0;
        double b = a / r_max;
        for (int k = 0; k < 32; k++) {
            VFHDensity[k] = 0.0;
            for (int i = 0; i < 8; i++) {
                double range = lidarRangeImage[k * 8 + i];
                if (range != INFINITY && range < r_max) {
                    double mki = a - b * range;
                    VFHDensity[k] += mki;
                }
            }
        }
    }

    void findValley(int k) {
        int iStart = -1, iEnd = 32 - 1;
        for (int i = k; i < 32; i++) {
            if (VFHDensity[i] < VFH_THRESHOLD) {
                if (iStart == -1) iStart = i;
            } else if (iStart != -1) {
                iEnd = i - 1;
                break;
            }
        }
        if (iStart != -1) {
            valleyPos[numValley] = VFHBearing[(iStart + iEnd) / 2];
            valleyWid[numValley] = (iEnd - iStart + 1) * (2.55 / 32);
            numValley++;
        }
        if (iStart != -1 && iEnd < 32 - 1) {
            findValley(iEnd + 1);
        }
    }

    void findValleys() {
        numValley = 0;
        findValley(0);
    }

    double selectVFHDirection(double theta_goal) {
        getVFH();
        findValleys();
        if (numValley == 0) return theta_goal;

        double min_diff = M_PI;
        double selected_angle = theta_goal;
        for (int i = 0; i < numValley; i++) {
            double diff = fabs(normalizeAngle(valleyPos[i] - theta_goal));
            if (diff < min_diff) {
                min_diff = diff;
                selected_angle = valleyPos[i];
            }
        }
        return selected_angle;
    }

private:
    bool readSensorData() {
        gpsLoc = gps->getValues();
        north = compass->getValues();
        lidarRangeImage = lidar->getRangeImage();
        return gpsLoc && north && lidarRangeImage;
    }

    // waypoint completion by removing reached waypoints and updating the waypoint array.
    // State_N0(), State_N2() when distance < FLYBY_RADIUS.
    void handleWaypointCompletion(double distance) {
        if (distance >= FLYBY_RADIUS) return;

        waypointFlags[currentWP] = true;
        delete[] waypoints[currentWP];
        for (int i = currentWP; i < numWP - 1; i++) {
            waypoints[i] = waypoints[i + 1];
            waypointFlags[i] = waypointFlags[i + 1];
        }
        numWP--;
        waypointFlags.pop_back();
        double** newWaypoints = new double*[numWP];
        for (int i = 0; i < numWP; i++) {
            newWaypoints[i] = waypoints[i];
        }
        delete[] waypoints;
        waypoints = newWaypoints;

        currentWP = findNearestWP(gpsLoc[0], gpsLoc[1]);
        if (numWP == 0) {
            navState = N3;
            numWP = 1;
            waypoints = new double*[1];
            waypoints[0] = new double[2]{0.0, 0.0};
            waypointFlags.assign(1, false);
            currentWP = 0;
        }
        prevLamT = getLamT();
        prevDLamT = 0.0;
    }

    // Sets a default waypoint at (0,0) when no waypoints exist.
    // transition() when numWP == 0.
    void NoWP() {
        if (waypoints) {
            for (int i = 0; i < numWP; i++) {
                delete[] waypoints[i];
            }
            delete[] waypoints;
        }
        numWP = 1;
        waypoints = new double*[1];
        waypoints[0] = new double[2]{0.0, 0.0};
        waypointFlags.assign(1, false);
        currentWP = 0;
    }

    // Checks for obstacles using range() and updates oa_state and navState.
    // State_N1(), State_N2(), State_N4() to OA.
    void OA() {
        unsigned char obstacle = range();
        switch (obstacle) {
            case U0_front:
                oa_state = S14;
                navState = N0;
                break;
            case U1_left:
                oa_state = S11;
                navState = N0;
                break;
            case U2_right:
                oa_state = S12;
                navState = N0;
                break;
            case U3_back_or_NAN:
                break;
        }
    }

    // Logic for N0 state, including waypoint completion and state transitions.
    // transition() when navState == N0.
    void State_N0(double distance, double &psi_c) {
        if (distance < FLYBY_RADIUS) {
            handleWaypointCompletion(distance);
            navState = (numWP == 0) ? N3 : N0;
        } else {
            navState = (numValley > 1) ? N4 : (distance > SWITCH_RADIUS) ? N1 : N2;
            psi_c = (navState == N1) ? PN_control() : (navState == N2) ? CLOS_control() : 0.0;
        }
    }

    // Logic for N1 state, including state transitions and obstacle checks.
    // transition() when navState == N1.
    void State_N1(double distance, double &psi_c) {
        if (distance <= SWITCH_RADIUS) {
            navState = N2;
        } else if (numValley > 1) {
            navState = N4;
        }
        OA();
        if (oa_state == S10) {
            psi_c = PN_control();
        }
    }

    // Logic for N2 state, including waypoint completion and obstacle checks.
    // transition() when navState == N2.
    void State_N2(double distance, double &psi_c) {
        if (distance > SWITCH_RADIUS) {
            navState = N1;
        } else if (distance < FLYBY_RADIUS) {
            handleWaypointCompletion(distance);
            navState = (numWP == 0) ? N3 : N0;
        } else if (numValley > 1) {
            navState = N4;
        }
        OA();
        if (oa_state == S10) {
            psi_c = CLOS_control();
        }
    }

    // Logic for N3 state, stopping the robot when the final waypoint is reached.
    // transition() when navState == N3.
    void State_N3(double distance, double &psi_c) {
        if (distance < FLYBY_RADIUS) {
            setMotorSpeeds(0.0);
            delete[] waypoints[0];
            delete[] waypoints;
            waypoints = nullptr;
            numWP = 0;
            waypointFlags.clear();
        } else {
            psi_c = CLOS_control();
        }
    }

    // Logic for N4 state, including VFH navigation and obstacle checks.
    // Used in: transition() when navState == N4.
    void State_N4(double distance, double &psi_c) {
        if (distance <= SWITCH_RADIUS) {
            navState = N2;
        } else if (numValley <= 1) {
            navState = N1;
        }
        OA();
        if (oa_state == S10) {
            double xT, yT;
            getTargetBodyCoords(xT, yT);
            double theta_wp = normalizeAngle(atan2(yT, xT));
            psi_c = selectVFHDirection(theta_wp);
            psi_c = max(-2.0, min(2.0, psi_c));
        }
    }

public:
    void transition() {

        // Receive new waypoints
        receiveWP();

        // Run OA state machine
        OAmyRobot::STATE_OA();
        if (oa_state != S10) return;

        // Update heading and distance
        getHeading();
        double distance = getDistanceToTarget();

        // case with no waypoints
        if (numWP == 0) {
            NoWP();
            navState = N3;
        }

        // Compute VFH data for navigation
        getVFH();
        findValleys();

        // Process navigation state
        double psi_c = 0.0;
        switch (navState) {
            case N0:
                State_N0(distance, psi_c);
                break;
            case N1:
                State_N1(distance, psi_c);
                break;
            case N2:
                State_N2(distance, psi_c);
                break;
            case N3:
                State_N3(distance, psi_c);
                break;
            case N4:
                State_N4(distance, psi_c);
                break;
            default:
                psi_c = 0.0;
                break;
        }

        // Apply motor speeds
        psi_c = max(-2.0, min(2.0, psi_c));
        setMotorSpeeds(psi_c);

        cout << "navState: " << navState << " | Waypoint: " << currentWP << " | Pos: (" << (gpsLoc ? gpsLoc[0] : 0.0) << ", " << (gpsLoc ? gpsLoc[1] : 0.0) << ")" << " | Distance: " << distance << " | NumValleys: " << static_cast<int>(numValley) << endl;
    }
};

