#include <algorithm>
#include <boost/bind.hpp>
#include <chrono>
#include <climits>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <iostream>
#include <vector>

#define L 0.04 // distance between body center and wheel center
#define r 0.01905 // wheel radius

#define pi30 0.1047197551196597705355242034774843062905347323976457118988037109375 // long double thirty = 30; long double mOne = -1; printf("%1.70Lf\n", (long double) acos(mOne) / thirty);
#define pi12 1.5707963267948965579989817342720925807952880859375 // long double two = 2; long double mOne = -1; printf("%1.70Lf\n", (long double) acos(mOne) / two);
#define pi 3.141592653589793115997963468544185161590576171875 // long double mOne = -1; printf("%1.70Lf\n", (long double) acos(mOne));
#define pi2 6.28318530717958623199592693708837032318115234375 // long double two = 2; long double mOne = -1; printf("%1.70Lf\n", (long double) two * acos(mOne));
#define sqrt3 1.732050807568877193176604123436845839023590087890625 // long double three = 3; printf("%1.70Lf\n", (long double) sqrt(three));
#define sqrt32 0.8660254037844385965883020617184229195117950439453125 // long double two = 2; long double three = 3; printf("%1.70Lf\n", (sqrt(three) / two));
#define Vmax 1.0 // set by experiments (m/s)
#define omegamax 50.0 // set by experiments (rad/s)
#define P 5.0 // PID
#define I 0.0005 // PID
#define stopDistance 0.001
#define stopAngle pi30

double Vx;
double Vy;
double Vxm;
double Vym;
double Vxw;
double Vyw;
double VxAbs;
double VyAbs;
double VxmTarget;
double VymTarget;
double VxwTarget;
double VywTarget;
double omegap;
double Vback;
double Vleft;
double Vright;
double VbackTarget;
double VleftTarget;
double VrightTarget;
double omegapL;
double omegapLVxm2;
double sqrtVym2;
double Vxm2;
double Vback3;
double Vleft3;
double Vright3;
double VxTarget;
double VyTarget;
double omegapTarget;
double x = 0;
double y = 0;
double xm = 0;
double ym = 0;
double xOffset = 0;
double yOffset = 0;
double xTarget = 0;
double yTarget = 0;
double xw = 0;
double yw = 0;
double xError;
double yError;
double xErrorI;
double yErrorI;
double theta = 0;
double thetaError;
double thetaErrorI;
double thetaOffset = 0;
double thetaTarget = 0;
double timeElapsed = 0;
double timeLast;
double timeNow = 0;
double backAngleElapsed;
double backAngleLast;
double backAngleNow = 0;
double leftAngleElapsed;
double leftAngleLast;
double leftAngleNow = 0;
double rightAngleElapsed;
double rightAngleLast;
double rightAngleNow = 0;
double phi;
double Vlow;
double Vhigh;
double scale;

class Point {

    public: double x;

    public: double y;

    private: static double w;

    public: void offset(double x, double y) {
        this->x += x;
        this->y += y;
    }

    public: void rotate(double theta) {
              w = (std::cos(theta) * this->y) + (std::sin(theta) * this->x);
        this->x = (std::cos(theta) * this->x) - (std::sin(theta) * this->y);
        this->y = w;
    }

    public: Point() {}

    public: Point(const Point& p) {
        x = p.x;
        y = p.y;
    }

    public: Point(double x, double y) {
        this->x = x;
        this->y = y;
    }

    public: Point(Point p, double xOffset, double yOffset) {
        x = p.x + xOffset;
        y = p.y + yOffset;
    }
};

std::vector<Point> points;
std::vector<double> angles;

enum Movements {
    MOVEMENT_DIRECT,
    MOVEMENT_DIRECT_M,
    MOVEMENT_DIRECT_W,
    MOVEMENT_NONE,
    MOVEMENT_ABSOLUTE_M,
    MOVEMENT_ABSOLUTE_W,
    MOVEMENT_BEZIER_W
};
Movements movement = MOVEMENT_NONE;

long long binomialCoefficient_c;
long long binomialCoefficient_i;

/**
 * source:
 * https://en.wikipedia.org/wiki/Binomial_coefficient
 * @param n the number that goes above
 * @param k the number that goes below
 * @return
 */
long long binomialCoefficient(long long n, long long k) {
    if ((k < 0) || (k > n))
        return 0;
    if ((k == 0) || (k == n))
        return 1;
    k = std::min(k, n - k);// take advantage of symmetry
    binomialCoefficient_c = 1;
    for (long binomialCoefficient_i = 0;
     binomialCoefficient_i < k; binomialCoefficient_i++)
        binomialCoefficient_c = binomialCoefficient_c
         * (n - binomialCoefficient_i) / (binomialCoefficient_i + 1);
    return binomialCoefficient_c;
}

double Bezier_t;
double Bezier_step;
long long Bezier_nT;
long long Bezier_nR;
long long Bezier_i;
double Bezier_B;
double Bezier_pX;
double Bezier_pY;
double Bezier_pTheta;

double Bezier_B_(double Bezier_n) {
    return ((double) binomialCoefficient(Bezier_n, Bezier_i))
         * std::pow((double) Bezier_t, (double) Bezier_i)
         * std::pow(1.0 - Bezier_t, (double) (Bezier_n - Bezier_i));
}

void Bezier() {
    Bezier_pX = 0;
    Bezier_pY = 0;
    for (Bezier_i = 0; Bezier_i <= Bezier_nT; Bezier_i++) {
        Bezier_B = Bezier_B_(Bezier_nT);
        Bezier_pX += points[Bezier_i].x * Bezier_B;
        Bezier_pY += points[Bezier_i].y * Bezier_B;
    }
    Bezier_pTheta = 0;
    for (Bezier_i = 0; Bezier_i <= Bezier_nR; Bezier_i++) {
        Bezier_pTheta += angles[Bezier_i] * Bezier_B_(Bezier_nR);
    }
}

/**
 * Resets the error values used by the PID controller.
 *
 * Should be used after a movement command if another one is fired
 * before or immediately after the previous one
 * and a continuous movement is not desired.
 */
void resetErrors() {
    xErrorI = 0;
    yErrorI = 0;
    thetaErrorI = 0;
}

/**
 * Calculates the mobile platform's velocities
 * relative to the mobile platform's frame
 * from the wheels velocities.
 */
void forwardKinematicsMobile() {
    Vleft3 = Vleft / 3.0;
    Vback3 = Vback / 3.0;
    Vright3 = Vright / 3.0;
    Vxm = (2 * Vback3) - Vleft3 - Vright3;
    Vym = (sqrt3 * Vright3) - (sqrt3 * Vleft3);
    omegap = (Vleft3 + Vback3 + Vright3) / L;
}

/**
 * Calculates the mobile platform's velocities
 * relative to the world's frame
 * from the wheels velocities.
 *
 * Also runs forwardKinematicsMobile().
 */
void forwardKinematicsWorld() {
    forwardKinematicsMobile();
    Vxw = (std::cos(theta) * Vxm) - (std::sin(theta) * Vym);
    Vyw = (std::cos(theta) * Vym) + (std::sin(theta) * Vxm);
}

/**
 * Generates a pseudo random number between 0 and maximumValue, inclusive.
 * @param maximumValue
 * @return
 */
double getRandom(double maximumValue) {
    return (((double) std::rand()) / ((double) RAND_MAX)) * maximumValue;
}

/**
 * Calculates the wheels velocities
 * from the mobile platform's velocities
 * relative to the mobile platform's frame.
 */
void inverseKinematicsMobile() {
    omegapL = omegap * L;
    sqrtVym2 = sqrt32 * Vym;
    Vxm2 = Vxm / 2.0;
    omegapLVxm2 = omegapL - Vxm2;
    Vleft  = omegapLVxm2 - sqrtVym2;
    Vback  = omegapL + Vxm;
    Vright = omegapLVxm2 + sqrtVym2;
}

/**
 * Calculates the wheels velocities
 * from the mobile platform's velocities
 * relative to the mobile platform's frame.
 *
 * Also runs inverseKinematicsMobile().
 */
void inverseKinematicsWorld() {
    Vxm = (std::cos(theta) * Vxw) + (std::sin(theta) * Vyw);
    Vym = (std::cos(theta) * Vyw) - (std::sin(theta) * Vxw);
    inverseKinematicsMobile();
}

bool isStopPose() {
    return (std::abs(xError) < stopDistance)
     && (std::abs(yError) < stopDistance)
     && (std::abs(thetaError) < stopAngle);
}

double normalizeRadian(double radian) {
    radian = fmod(radian, pi2);
    if (radian < 0) radian += pi2;
    return radian;
}

namespace gazebo {

    class OmniPlatformPlugin : public ModelPlugin {

        private: physics::JointPtr backJoint;

        private: physics::JointPtr leftJoint;

        // Pointer to the model
        private: physics::ModelPtr model;

        private: physics::JointPtr rightJoint;

        // Pointer to the update event connection
        private: event::ConnectionPtr updateConnection;

        private: physics::WorldPtr world;

        /**
         * Movement for desired pose in mobile platform's frame.
         *
         * Translates the desired pose in the mobile platform's frame
         * to the world's frame and moves according to the world's frame.
         * Keep in mind that how the mobile platform reaches the target theta
         * will affect the platform's position in it's own frame,
         * and that position can't be predicted. Also remember
         * that you can change the value of the platform's pose in either frame.
         * To be executed by the communications module.
         *
         * @param x
         * @param y
         * @param theta
         */
        public: void fireMovementAbsoluteM(double x, double y, double theta) {
            // distance to be traversed in the mobile platform's frame
            xTarget = x - xm;
            yTarget = y - ym;
            // distance rotated to the direction
            // to be traversed in the world's frame
            x = (std::cos(theta) * xTarget) - (std::sin(theta) * yTarget);
            y = (std::cos(theta) * yTarget) + (std::sin(theta) * xTarget);
            // rotated distance added to current world's frame position
            xTarget = xw + x;
            yTarget = yw + y;
            thetaTarget = theta;
            movement = MOVEMENT_ABSOLUTE_W;
            updateIndicator();
        }

        /**
         * Random movement for desired pose in mobile platform's frame
         *
         * At the end of the movement, the mobile platform's x, y and theta
         * will be the same as the parameters. However, the position
         * in the world frame will depend on how theta will vary
         * from it's current value to the target value, and can't be predicted.
         * It's preferred to use fireMovementPoseM(),
         * unless you know what you're doing.
         * To be executed by the communications module.
         *
         * @param x
         * @param y
         * @param theta
         */
        public: void fireMovementAbsoluteMRaw(double x, double y, double theta) {
            xTarget = x;
            yTarget = y;
            thetaTarget = theta;
            movement = MOVEMENT_ABSOLUTE_M;
            updateIndicator();
        }

        /**
         * Movement for desired pose in world frame.
         *
         * To be executed by the communications module.
         *
         * @param x
         * @param y
         * @param theta
         */
        public: void fireMovementAbsoluteW(double x, double y, double theta) {
            xTarget = x;
            yTarget = y;
            thetaTarget = theta;
            movement = MOVEMENT_ABSOLUTE_W;
            updateIndicator();
        }

        /**
         * Incremental movement based on two Bézier curves
         * relative to the platform's frame.
         *
         * The curve may be translated such that the first control point
         * and angle coincides with the current platform's position and angle.
         *
         * The Bézier function's <code>t</code> variable, by default, iterates
         * between 0 and 1 to generate the curve. <code>t</code>'s step value
         * is defined by the user, and influences the time the platform
         * will take to execute the movement, which is equal to the step value
         * divided by the iteration time. If the movement time is too short
         * (which depends on the movement length and complexity), the platform
         * will make it's best effort to execute the movement. Therefore,
         * the longer the movement time (the shorter the step value), the closer
         * the platform will be to the desired movement.
         *
         * The platform's angle is also controlled by a Bézier curve. However,
         * the algorithm has been modified to be one dimensional, which means
         * its input are control values instead of control points.
         * If a constant angle is desired, input a vector with a single value
         * equal to the angle.
         *
         * @param step Bézier funcion's <code>t</code> variable's increment value.
         * @param points control points for the translation movement
         * @param angles control angles for the rotation movement
         * @param offsetT whether to offset the control points to the starting platform's position
         * @param offsetR whether to offset the control angles to the starting platform's angle
         */
        public: void fireMovementBezierM(std::vector<Point> * points,
                std::vector<double> * angles, double step,
                bool offsetT, bool offsetR) {
            if (points->empty() || angles->empty()) return;
            ::points.clear();
            ::angles.clear();
            Bezier_nT = points->size() - 1;
            Bezier_nR = angles->size() - 1;
            for (Bezier_i = 0; Bezier_i <= Bezier_nT; Bezier_i++) {
                ::points.push_back(Point(points->at(Bezier_i)));
                ::points[Bezier_i].offset(-xw, -yw);
                ::points[Bezier_i].rotate(theta);
                ::points[Bezier_i].offset(xw, yw);
            }
            if (offsetT) {
                xOffset = xw - ::points[0].x;
                yOffset = yw - ::points[0].y;
            } else {
                xOffset = 0;
                yOffset = 0;
            }
            for (Bezier_i = 0; Bezier_i <= Bezier_nT; Bezier_i++) {
                ::points[Bezier_i].offset(xOffset, yOffset);
            }
            if (offsetR) {
                thetaOffset = pi - normalizeRadian((*angles)[0] + pi - theta);
            } else {
                thetaOffset = 0;
            }
            for (Bezier_i = 0; Bezier_i <= Bezier_nR; Bezier_i++) {
                ::angles.push_back(angles->at(Bezier_i) + thetaOffset);
            }
            Bezier_t = 0;
            Bezier_step = step;
            movement = MOVEMENT_BEZIER_W;
            updateIndicator();
        }

        /**
         * Incremental movement based on two Bézier curves relative to the world's frame.
         *
         * The curve may be translated such that the first control point
         * and angle coincides with the current platform's position and angle.
         *
         * The Bézier function's <code>t</code> variable, by default, iterates
         * between 0 and 1 to generate the curve. <code>t</code>'s step value
         * is defined by the user, and influences the time the platform
         * will take to execute the movement, which is equal to the step value
         * divided by the iteration time. If the movement time is too short
         * (which depends on the movement length and complexity), the platform
         * will make it's best effort to execute the movement. Therefore,
         * the longer the movement time (the shorter the step value), the closer
         * the platform will be to the desired movement.
         *
         * The platform's angle is also controlled by a Bézier curve. However,
         * the algorithm has been modified to be one dimensional, which means
         * its input are control values instead of control points.
         * If a constant angle is desired, input a vector with a single value
         * equal to the angle.
         *
         * @param step Bézier funcion's <code>t</code> variable's increment value.
         * @param points control points for the translation movement
         * @param angles control angles for the rotation movement
         * @param offsetT whether to offset the control points to the starting platform's position
         * @param offsetR whether to offset the control angles to the starting platform's angle
         */
        public: void fireMovementBezierW(std::vector<Point> * points,
                std::vector<double> * angles, double step,
                bool offsetT, bool offsetR) {
            if (points->empty() || angles->empty()) return;
            ::points.clear();
            ::angles.clear();
            Bezier_nT = points->size() - 1;
            Bezier_nR = angles->size() - 1;
            if (offsetT) {
                xOffset = xw - (*points)[0].x;
                yOffset = yw - (*points)[0].y;
            } else {
                xOffset = 0;
                yOffset = 0;
            }
            if (offsetR) {
                thetaOffset = pi - normalizeRadian((*angles)[0] + pi - theta);
            } else {
                thetaOffset = 0;
            }
            for (Bezier_i = 0; Bezier_i <= Bezier_nT; Bezier_i++) {
                ::points.push_back(Point(points->at(Bezier_i), xOffset, yOffset));
            }
            for (Bezier_i = 0; Bezier_i <= Bezier_nR; Bezier_i++) {
                ::angles.push_back(angles->at(Bezier_i) + thetaOffset);
            }
            Bezier_t = 0;
            Bezier_step = step;
            movement = MOVEMENT_BEZIER_W;
            updateIndicator();
        }

        /**
         * Moves the platform with fixed translation and rotation speeds
         * relative to the mobile platform's frame but translated
         * to the world's frame. This results in a world's frame movement
         * rotated according to the initial angle between frames.
         *
         * To be executed by the communications module.
         *
         * @param Vleft
         * @param Vback
         * @param Vright
         */
        public: void fireMovementDirectHybrid(double Vxm, double Vym, double omegap) {
            VxTarget = (std::cos(theta) * Vxm) - (std::sin(theta) * Vym);
            VyTarget = (std::cos(theta) * Vym) + (std::sin(theta) * Vxm);
            omegapTarget = omegap;
            movement = MOVEMENT_DIRECT_W;
        }

        /**
         * Moves the platform with fixed translation and rotation speeds,
         * relative to the mobile platform's frame. Does not transform
         * the speeds from the mobile platform's frame to the world frame.
         * This means that random changes in the platform's angle
         * will not change it's movement, as the mobile platform's frame
         * rotates with the platform.
         *
         * To be executed by the communications module.
         *
         * @param Vleft
         * @param Vback
         * @param Vright
         */
        public: void fireMovementDirectMobile(double Vxm, double Vym, double omegap) {
            VxTarget = Vxm;
            VyTarget = Vym;
            omegapTarget = omegap;
            movement = MOVEMENT_DIRECT_M;
        }

        /**
         * Sets the wheels' speeds, in m/s.
         *
         * To be executed by the communications module.
         * @param Vleft
         * @param Vback
         * @param Vright
         */
        public: void fireMovementDirectWheel(double Vleft, double Vback, double Vright) {
            VleftTarget  = Vleft ;
            VbackTarget  = Vback ;
            VrightTarget = Vright;
            movement = MOVEMENT_DIRECT;
        }

        /**
         * Moves the platform with fixed translation and rotation speeds,
         * relative to the world's frame.
         *
         * To be executed by the communications module.
         *
         * @param Vleft
         * @param Vback
         * @param Vright
         */
        public: void fireMovementDirectWorld(double Vxw, double Vyw, double omegap) {
            VxTarget     = Vxw   ;
            VyTarget     = Vyw   ;
            omegapTarget = omegap;
            movement = MOVEMENT_DIRECT_W;
        }

        /**
         * Movement of a given distance and angle of rotation
         * in the mobile platform's frame
         *
         * Translates the movement in the mobile platform's frame
         * to the world's frame and moves according to the world's frame.
         * Keep in mind that how the mobile platform reaches the target theta
         * will affect the platform's position in it's own frame,
         * and that position can't be predicted. Also remember
         * that you can change the value of the platform's pose in either frame.
         * To be executed by the communications module.
         *
         * @param x
         * @param y
         * @param theta
         */
        public: void fireMovementRelativeM(double x, double y, double theta) {
            // distance to be traversed in the mobile platform's frame
            xTarget = x;
            yTarget = y;
            // distance rotated to the direction
            // to be traversed in the world's frame
            x = (std::cos(theta) * xTarget) - (std::sin(theta) * yTarget);
            y = (std::cos(theta) * yTarget) + (std::sin(theta) * xTarget);
            // rotated distance added to current world's frame position
            xTarget = xw + x;
            yTarget = yw + y;
            thetaTarget = normalizeRadian(::theta + theta);
            movement = MOVEMENT_ABSOLUTE_W;
            updateIndicator();
        }

        /**
         * Random movement of a given distance and angle of rotation
         * in the mobile platform's frame
         *
         * At the end of the movement, the mobile platform will have traversed
         * in x and y and rotated by theta, relative to its frame.
         * However, the position in the world frame will depend
         * on how theta will vary from it's current value to the target value,
         * and can't be predicted. It's preferred to use fireMovementPoseM(),
         * unless you know what you're doing.
         * To be executed by the communications module.
         *
         * @param x
         * @param y
         * @param theta
         */
        public: void fireMovementRelativeMRaw(double x, double y, double theta) {
            xTarget = xm + x;
            yTarget = ym + y;
            thetaTarget = normalizeRadian(::theta + theta);
            movement = MOVEMENT_ABSOLUTE_M;
            updateIndicator();
        }

        /**
         * Movement of a given distance and angle of rotation
         * relative to the world frame.
         *
         * To be executed by the communications module.
         *
         * @param x
         * @param y
         * @param theta
         */
        public: void fireMovementRelativeW(double x, double y, double theta) {
            xTarget = xw + x;
            yTarget = yw + y;
            thetaTarget = normalizeRadian(::theta + theta);
            movement = MOVEMENT_ABSOLUTE_W;
            updateIndicator();
        }

        public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
            // Store the pointer to the model
            this->model = _parent;

            this-> leftJoint = model->GetJoint("left_joint");
            this-> backJoint = model->GetJoint("back_joint");
            this->rightJoint = model->GetJoint("right_joint");

            this->world = _parent->GetWorld();
            //this->indicator = this->world->GetModel("wood_cube_2_5cm");
            //this->indicatorPose.reset(new math::Pose());
            //this->indicator->SetGravityMode(false);
            //this->indicator->Fini();

            std::srand(std::time(0));

            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                    boost::bind(&OmniPlatformPlugin::OnUpdate, this, _1));
        }

        private: void odometry() {
            timeLast = timeNow;
            timeNow = this->world->GetSimTime().Double();
            timeElapsed = timeNow - timeLast;
            Vleft  = this-> leftJoint->GetVelocity(0) * r;
            Vback  = this-> backJoint->GetVelocity(0) * r;
            Vright = this->rightJoint->GetVelocity(0) * r;
            theta = normalizeRadian(model->GetRelativePose().rot.GetYaw()); // simulation theta
            forwardKinematicsWorld();
            xm += Vxm * timeElapsed;
            ym += Vym * timeElapsed;
            xw += Vxw * timeElapsed;
            //xw = model->GetRelativePose().pos.x; // simulation x
            yw += Vyw * timeElapsed;
            //yw = model->GetRelativePose().pos.y; // simulation y
        }

        // Called by the world update start event
        public: void OnUpdate(const common::UpdateInfo & /*_info*/) {
            odometry();
            switch (movement) {
                case MOVEMENT_ABSOLUTE_M:
                    xError = xTarget - xm;
                    yError = yTarget - ym;
                    // method parameter: theta plus shift to place thetaTarget at pi
                    thetaError = pi - normalizeRadian(theta + pi - thetaTarget);
                    if (isStopPose()) {
                        movement = MOVEMENT_NONE;
                        break;
                    }
                    xErrorI += xError;
                    yErrorI += yError;
                    thetaErrorI += thetaError;
                    Vxm = (xError * P) + (xErrorI * I);
                    Vym = (yError * P) + (yErrorI * I);
                    VxAbs = std::abs(Vxm);
                    VyAbs = std::abs(Vym);
                    if ((VxAbs > Vmax) || (VyAbs > Vmax)) {
                        scale = Vmax / ((VxAbs > VyAbs) ? VxAbs : VyAbs);
                        Vxm *= scale;
                        Vym *= scale;
                    }
                    omegap = (thetaError * P) + (thetaErrorI * I);
                    if (omegap > omegamax) {
                        omegap = omegamax;
                    }
                    inverseKinematicsMobile();
                    break;
                case MOVEMENT_BEZIER_W:
                    if (Bezier_t <= 1) {
                        Bezier();
                        xTarget = Bezier_pX;
                        yTarget = Bezier_pY;
                        thetaTarget = normalizeRadian(Bezier_pTheta);
                        Bezier_t += Bezier_step;
                        updateIndicator();
                    } else {
                        movement = MOVEMENT_NONE;
                    }
                    // break; // fall through intended
                case MOVEMENT_ABSOLUTE_W:
                    xError = xTarget - xw;
                    yError = yTarget - yw;
                    // method parameter: theta plus shift to place thetaTarget at pi
                    thetaError = pi - normalizeRadian(theta + pi - thetaTarget);
                    if (isStopPose() && (movement == MOVEMENT_ABSOLUTE_W)) {
                        movement = MOVEMENT_NONE;
                        break;
                    }
                    xErrorI += xError;
                    yErrorI += yError;
                    thetaErrorI += thetaError;
                    Vxw = (xError * P) + (xErrorI * I);
                    Vyw = (yError * P) + (yErrorI * I);
                    VxAbs = std::abs(Vxw);
                    VyAbs = std::abs(Vyw);
                    if ((VxAbs > Vmax) || (VyAbs > Vmax)) {
                        scale = Vmax / ((VxAbs > VyAbs) ? VxAbs : VyAbs);
                        Vxw *= scale;
                        Vyw *= scale;
                    }
                    omegap = (thetaError * P) + (thetaErrorI * I);
                    if (omegap > omegamax) {
                        omegap = omegamax;
                    }
                    inverseKinematicsWorld();
                    break;
                case MOVEMENT_DIRECT:
                    Vleft  = VleftTarget ;
                    Vback  = VbackTarget ;
                    Vright = VrightTarget;
                    break;
                case MOVEMENT_DIRECT_M:
                    Vxm    = VxTarget    ;
                    Vym    = VyTarget    ;
                    omegap = omegapTarget;
                    inverseKinematicsMobile();
                    break;
                case MOVEMENT_DIRECT_W:
                    Vxw    = VxTarget    ;
                    Vyw    = VyTarget    ;
                    omegap = omegapTarget;
                    inverseKinematicsWorld();
                    break;
                default:
                    xErrorI = 0;
                    yErrorI = 0;
                    thetaErrorI = 0;
                    Vleft = 0;
                    Vback = 0;
                    Vright = 0;
            }
             leftJoint->SetVelocity(0, Vleft / r);
             backJoint->SetVelocity(0, Vback / r);
            rightJoint->SetVelocity(0, Vright / r);
        }

        /**
         * Simulation purposes only.
         */
        private: void updateIndicator() {
//            indicatorPose->Set(xTarget, yTarget, 0.05, 0, 0, thetaTarget);
//            indicator->SetRelativePose(*indicatorPose);
        }
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(OmniPlatformPlugin)
}

// C++ bug: when things are declared as 'static' or 'const',
// they lose their references, and it's only possible to read from them
// to restore their references, they must be decladed again like this:
double Point::w;
