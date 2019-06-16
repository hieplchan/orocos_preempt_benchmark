#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>
#include <kdl/chain.hpp>

#include <limits.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>

#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>

using namespace KDL;
using namespace std;

void inverse_dynamic()
{
  clock_t tStart = clock();

	Vector gravity(0, 0, -10);

	Chain RRBotKdl = Chain();

  //Link inertia of robot arm (Ixx, Iyy, Izz, Ixy, Ixz, Iyz)
	RigidBodyInertia inert1 = RigidBodyInertia(0, Vector(0, 0, 0), RotationalInertia(0, 0.35, 0, 0, 0, 0));
	RigidBodyInertia inert2 = RigidBodyInertia(17.4, Vector(-0.3638, 0.006, 0.2275), RotationalInertia(0.13, 0.524, 0.539, 0, 0, 0));
	RigidBodyInertia inert3 = RigidBodyInertia(4.8, Vector(-0.0203, -0.0141, 0.070), RotationalInertia(0.066, 0.086, 0.0125, 0, 0, 0));
	RigidBodyInertia inert4 = RigidBodyInertia(0.82, Vector(0, 0.019, 0), RotationalInertia(10.8e-3, 10.3e-3, 10.8e-3, 0, 0, 0));
	RigidBodyInertia inert5 = RigidBodyInertia(0.34, Vector(0, 0, 0), RotationalInertia(0.3e-3, 0.4e-3, 0.3e-3, 0, 0, 0));
	RigidBodyInertia inert6 = RigidBodyInertia(0.09, Vector(0, 0, 0.032), RotationalInertia(0.15e-3, 0.15e-3, 0.04e-3, 0, 0, 0));

  //D-H parameters of robot arm (a, alpha, d, theta)
	Frame frame1 = Frame::DH(0, M_PI/2, 0, 0);
	Frame frame2 = Frame::DH(0.4318, 0, 0, 0);
	Frame frame3 = Frame::DH(0.0203, -M_PI/2, 0.15005, 0);
	Frame frame4 = Frame::DH(0, M_PI/2, 0.4318, 0);
	Frame frame5 = Frame::DH(0, -M_PI/2, 0, 0);
	Frame frame6 = Frame::DH(0, 0, 0, 0);

  //Rotation or translation of each joint
	Joint joint1(Joint::RotZ);
	Joint joint2(Joint::RotZ);
	Joint joint3(Joint::RotZ);
	Joint joint4(Joint::RotZ);
	Joint joint5(Joint::RotZ);
	Joint joint6(Joint::RotZ);

  //Add links to Robot arm
	RRBotKdl.addSegment(Segment(joint1, frame1, inert1));
	RRBotKdl.addSegment(Segment(joint2, frame2, inert2));
	RRBotKdl.addSegment(Segment(joint3, frame3, inert3));
	RRBotKdl.addSegment(Segment(joint4, frame4, inert4));
	RRBotKdl.addSegment(Segment(joint5, frame5, inert5));
	RRBotKdl.addSegment(Segment(joint6, frame6, inert6));

  //Joint angle position
	JntArray jointAngles = JntArray(6);
	jointAngles(0) = M_PI/2;       // Joint 1
	jointAngles(1) = M_PI/2;       // Joint 2
	jointAngles(2) = M_PI/2;       // Joint 3
	jointAngles(3) = M_PI/2;       // Joint 4
	jointAngles(4) = M_PI/2;       // Joint 5
	jointAngles(5) = M_PI/2;       // Joint 6

  //Joint angle velocity
	JntArray jointVel = JntArray(6);
	jointVel(0) = 1;
	jointVel(1) = 1;
	jointVel(2) = 1;
	jointVel(3) = 1;
	jointVel(4) = 1;
	jointVel(5) = 1;

  //Joint angle accelerate
	JntArray jointAcc = JntArray(6);
	jointAcc(0) = 2;
	jointAcc(1) = 2;
	jointAcc(2) = 2;
	jointAcc(3) = 2;
	jointAcc(4) = 2;
	jointAcc(5) = 2;

  //Joint added moment or external force (mass, ...)
	Wrenches jnt_wrenches;
	//jnt_wrenches.push_back(Wrench(Vector(0, 0, -10), Vector(0, 0, 0)));
	jnt_wrenches.push_back(Wrench());
	jnt_wrenches.push_back(Wrench());
	jnt_wrenches.push_back(Wrench());
	jnt_wrenches.push_back(Wrench());
	jnt_wrenches.push_back(Wrench());
	jnt_wrenches.push_back(Wrench());

  //Moment of each joint
	JntArray jointforce = JntArray(6);

	//Inverse Dynamic
	ChainIdSolver_RNE gcSolver = ChainIdSolver_RNE(RRBotKdl, gravity);
	int ret_gc = gcSolver.CartToJnt(jointAngles, jointVel, jointAcc, jnt_wrenches, jointforce);

	//cout << jointforce(0) << endl;
  printf("%d\n", (int)(clock() - tStart));
}

void *thread_func(void *data)
{
	//clock_t tStart = clock();


	for (int i = 0; i < 1000000; i++)
	{
		inverse_dynamic();
	}

	inverse_dynamic();

	//printf("%d us\n", (int)(clock() - tStart));
}

int main(int argc, char* argv[])
{
        struct sched_param param;
        pthread_attr_t attr;
        pthread_t thread;
        int ret;

        //Lock all memory of current process
        if(mlockall(MCL_CURRENT) == -1) {
                printf("mlockall failed: %m\n");
                exit(-2);
        }

        /* Initialize pthread attributes (default values) */
        ret = pthread_attr_init(&attr);
        if (ret) {
                printf("init pthread attributes failed\n");
                goto out;
        }

        /* Set a specific stack size (can be modified for different application)  */
        ret = pthread_attr_setstacksize(&attr, 1000000);
        if (ret) {
            printf("pthread setstacksize failed\n");
            goto out;
        }

        /* Set scheduler policy and priority of pthread */
        ret = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
        if (ret) {
                printf("pthread setschedpolicy failed\n");
                goto out;
        }
        param.sched_priority = 99;
        ret = pthread_attr_setschedparam(&attr, &param);
        if (ret) {
                printf("pthread setschedparam failed\n");
                goto out;
        }

        /* Use scheduling parameters of attr */
        ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
        if (ret) {
                printf("pthread setinheritsched failed\n");
                goto out;
        }

        /* Create a pthread with specified attributes */
        ret = pthread_create(&thread, &attr, thread_func, NULL);
        if (ret) {
                printf("create pthread failed\n");
                goto out;
        }

        /* Join the thread and wait until it is done */
        ret = pthread_join(thread, NULL);
        if (ret)
                printf("join pthread failed: %m\n");

out:
        return ret;
}
