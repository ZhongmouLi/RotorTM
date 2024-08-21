#include <iostream> 
#include <gtest/gtest.h>
#include <memory>
#include <cmath>
#include <cstdlib>
#include <random>
#include "rotor_tm_sim/lib_rigidbody.hpp"


class rotorTMRigidBodyTest : public ::testing::Test
{
public:

rotorTMRigidBodyTest(){
    Eigen::Matrix3d m_inertia = Eigen::Matrix3d::Identity(3,3);

    double mass =1;

    MassProperty mass_property = {mass, m_inertia};

    double dt = 0.01;

    ptr_rigidbody = std::make_shared<RigidBody>(mass_property, dt);
}

~rotorTMRigidBodyTest(){
}

protected:
    std::shared_ptr<RigidBody> ptr_rigidbody;
};


TEST_F(rotorTMRigidBodyTest, checkGTest){
    // test if gTest is well integrated
    ASSERT_TRUE(true);
}

TEST_F(rotorTMRigidBodyTest, checkInstanceClass){
    // test if instance is created
    ASSERT_TRUE(ptr_rigidbody!=nullptr);
}

// input force that compensates gravity
TEST_F(rotorTMRigidBodyTest, calHoverEqulibirum){

    // define force to compensite gravity and zero torque
    const Eigen::Vector3d force(0, 0, 9.8);

    const Eigen::Vector3d torque(0, 0, 0);
    
    const Wrench mav_wrench = {force, torque};

    // input force and torque
    // ptr_rigidbody->InputForce(force);
    // ptr_rigidbody->InputTorque(torque);
    ptr_rigidbody->InputWrench(mav_wrench);


    // do one integration
    ptr_rigidbody->DoOneStepInt();

    // check position = 0.5at^2
    Pose pose;

    // 

    pose =  ptr_rigidbody->pose();

    EXPECT_FLOAT_EQ(pose.post[0], 0); 
    EXPECT_FLOAT_EQ(pose.post[1], 0); 
    EXPECT_FLOAT_EQ(pose.post[2], 0);   

    // check vel = gt
    Vels vels;
    // 

    vels = ptr_rigidbody->vels();

    EXPECT_FLOAT_EQ(vels.linear_vel[0], 0); 
    EXPECT_FLOAT_EQ(vels.linear_vel[1], 0); 
    EXPECT_FLOAT_EQ(vels.linear_vel[2], 0);  

    // check acc = 0
    Accs accs;
    // 
    accs =  ptr_rigidbody->accs();
    
    EXPECT_FLOAT_EQ(accs.linear_acc[0], 0); 
    EXPECT_FLOAT_EQ(accs.linear_acc[1], 0); 
    EXPECT_FLOAT_EQ(accs.linear_acc[2], 0);  


    // check attitude = 0
    EXPECT_FLOAT_EQ(pose.att.x(), 0); 
    EXPECT_FLOAT_EQ(pose.att.y(), 0); 
    EXPECT_FLOAT_EQ(pose.att.z(), 0);  
    EXPECT_FLOAT_EQ(pose.att.w(), 1);  

    // check bodyrate =0
    EXPECT_FLOAT_EQ(vels.bodyrate[0], 0); 
    EXPECT_FLOAT_EQ(vels.bodyrate[1], 0); 
    EXPECT_FLOAT_EQ(vels.bodyrate[2], 0);  

    // check angular_acc =0
    EXPECT_FLOAT_EQ(accs.angular_acc[0], 0); 
    EXPECT_FLOAT_EQ(accs.angular_acc[1], 0); 
    EXPECT_FLOAT_EQ(accs.angular_acc[2], 0);  



    //check time step = dt
    double step_current = ptr_rigidbody->timestep();
    EXPECT_FLOAT_EQ(step_current, 0.01);  
}


// input force in z direction
TEST_F(rotorTMRigidBodyTest, applyForce4Zdirection){

    // define force in Z direction and zero torque
    const Eigen::Vector3d force(0, 0, 15);

    const Eigen::Vector3d torque(0, 0, 0);
    
    const Wrench mav_wrench = {force, torque};

    // input force and torque
    // ptr_rigidbody->InputForce(force);
    // ptr_rigidbody->InputTorque(torque);
     ptr_rigidbody->InputWrench(mav_wrench);

    // do one integration
    ptr_rigidbody->DoOneStepInt();


    // obtain pose, vels, accs
    Pose pose =  ptr_rigidbody->pose();
    Vels vels = ptr_rigidbody->vels();    
    Accs accs = ptr_rigidbody->accs();
    double step_current = ptr_rigidbody->timestep();
    
    
    // check position = 0.5at^2
    EXPECT_FLOAT_EQ(pose.post[0], 0); 
    EXPECT_FLOAT_EQ(pose.post[1], 0); 
    EXPECT_FLOAT_EQ(pose.post[2], 0.00026);    

    // check vel = at
    EXPECT_FLOAT_EQ(vels.linear_vel[0], 0); 
    EXPECT_FLOAT_EQ(vels.linear_vel[1], 0); 
    EXPECT_FLOAT_EQ(vels.linear_vel[2], 0.05199999999999999);  


    // check acc = a
    EXPECT_FLOAT_EQ(accs.linear_acc[0], 0); 
    EXPECT_FLOAT_EQ(accs.linear_acc[1], 0); 
    EXPECT_FLOAT_EQ(accs.linear_acc[2], 5.2);  


    // check attitude = 0
    EXPECT_FLOAT_EQ(pose.att.x(), 0); 
    EXPECT_FLOAT_EQ(pose.att.y(), 0); 
    EXPECT_FLOAT_EQ(pose.att.z(), 0);  
    EXPECT_FLOAT_EQ(pose.att.w(), 1);  

    // check bodyrate =0
    EXPECT_FLOAT_EQ(vels.bodyrate[0], 0); 
    EXPECT_FLOAT_EQ(vels.bodyrate[1], 0); 
    EXPECT_FLOAT_EQ(vels.bodyrate[2], 0);  

    // check angular_acc =0
    EXPECT_FLOAT_EQ(accs.angular_acc[0], 0); 
    EXPECT_FLOAT_EQ(accs.angular_acc[1], 0); 
    EXPECT_FLOAT_EQ(accs.angular_acc[2], 0);   


    //check time step = dt
    EXPECT_FLOAT_EQ(step_current, 0.01);  
}



TEST_F(rotorTMRigidBodyTest, applyRandomForceOneStep){

    // define radom force and zero torque
    const Eigen::Vector3d force = Eigen::Vector3d::Random();

    const Eigen::Vector3d torque(0, 0, 0);
    
    const Wrench mav_wrench = {force, torque};

    // input force and torque
    // ptr_rigidbody->InputForce(force);
    // ptr_rigidbody->InputTorque(torque);
     ptr_rigidbody->InputWrench(mav_wrench);

    // do one integration
    ptr_rigidbody->DoOneStepInt();

    // obtain pose, vels, accs
    Pose pose =  ptr_rigidbody->pose();
    Vels vels = ptr_rigidbody->vels();    
    Accs accs = ptr_rigidbody->accs();
    double step_current = ptr_rigidbody->timestep();

    EXPECT_FLOAT_EQ(pose.post[0], 0.5 * (force[0]) * 0.01 *0.01); 
    EXPECT_FLOAT_EQ(pose.post[1], 0.5 * (force[1]) * 0.01 *0.01); 
    EXPECT_FLOAT_EQ(pose.post[2], 0.5 * (force[2]-9.8) * 0.01 *0.01); 

    // check vel = at
    EXPECT_FLOAT_EQ(vels.linear_vel[0], (force[0]) *0.01);
    EXPECT_FLOAT_EQ(vels.linear_vel[1], (force[1]) *0.01);
    EXPECT_FLOAT_EQ(vels.linear_vel[2], (force[2]-9.8) *0.01);

    // check acc = a
    EXPECT_FLOAT_EQ(accs.linear_acc[0], force[0]); 
    EXPECT_FLOAT_EQ(accs.linear_acc[1], force[1]); 
    EXPECT_FLOAT_EQ(accs.linear_acc[2], (force[2]-9.8));  


    // check attitude = 0
    EXPECT_FLOAT_EQ(pose.att.x(), 0); 
    EXPECT_FLOAT_EQ(pose.att.y(), 0); 
    EXPECT_FLOAT_EQ(pose.att.z(), 0);  
    EXPECT_FLOAT_EQ(pose.att.w(), 1);  

    // check bodyrate =0

    EXPECT_FLOAT_EQ(vels.bodyrate[0], 0); 
    EXPECT_FLOAT_EQ(vels.bodyrate[1], 0); 
    EXPECT_FLOAT_EQ(vels.bodyrate[2], 0);  

    // check angular_acc =0
    EXPECT_FLOAT_EQ(accs.angular_acc[0], 0); 
    EXPECT_FLOAT_EQ(accs.angular_acc[1], 0); 
    EXPECT_FLOAT_EQ(accs.angular_acc[2], 0);  


    //check time step = dt
    EXPECT_FLOAT_EQ(step_current, 0.01);  
}

TEST_F(rotorTMRigidBodyTest, applyIdentityForceTwoSteps){

    // define radom force and zero torque
    // const Eigen::Vector3d force = Eigen::Vector3d::Random();
    const Eigen::Vector3d force(1, 1, 1);

    const Eigen::Vector3d torque(0, 0, 0);
    
    const Wrench mav_wrench = {force, torque};

    // input force and torque
    // ptr_rigidbody->InputForce(force);
    // ptr_rigidbody->InputTorque(torque);
    ptr_rigidbody->InputWrench(mav_wrench);

    // do two steps integration
    const double dt = 0.01;
    for(double t=0.0 ; t<2*dt ; t+= dt)
    {
            ptr_rigidbody->DoOneStepInt();
            // printf("current step is %.3f", t);
    }
    
    // obtain pose, vels, accs
    Pose pose =  ptr_rigidbody->pose();
    Vels vels = ptr_rigidbody->vels();    
    Accs accs = ptr_rigidbody->accs();
    double step_current = ptr_rigidbody->timestep();


    // check position = 0.5at^2
    EXPECT_FLOAT_EQ(pose.post[0], 0.5 * (force[0]) * pow(0.02,2)); 
    EXPECT_FLOAT_EQ(pose.post[1], 0.5 * (force[1]) * pow(0.02,2)); 
    EXPECT_FLOAT_EQ(pose.post[2], 0.5 * (force[2]-9.8) * pow(0.02,2)); 

    // check vel = at
    EXPECT_FLOAT_EQ(vels.linear_vel[0], (force[0]) *0.02);
    EXPECT_FLOAT_EQ(vels.linear_vel[1], (force[1]) *0.02);
    EXPECT_FLOAT_EQ(vels.linear_vel[2], (force[2]-9.8) *0.02);

    // check acc = a
    EXPECT_FLOAT_EQ(accs.linear_acc[0], force[0]); 
    EXPECT_FLOAT_EQ(accs.linear_acc[1], force[1]); 
    EXPECT_FLOAT_EQ(accs.linear_acc[2], (force[2]-9.8));  


    // check attitude = 0
    EXPECT_FLOAT_EQ(pose.att.x(), 0); 
    EXPECT_FLOAT_EQ(pose.att.y(), 0); 
    EXPECT_FLOAT_EQ(pose.att.z(), 0);  
    EXPECT_FLOAT_EQ(pose.att.w(), 1);  

    // check bodyrate =0
    EXPECT_FLOAT_EQ(vels.bodyrate[0], 0); 
    EXPECT_FLOAT_EQ(vels.bodyrate[1], 0); 
    EXPECT_FLOAT_EQ(vels.bodyrate[2], 0);  


    // check angular_acc =0
    EXPECT_FLOAT_EQ(accs.angular_acc[0], 0); 
    EXPECT_FLOAT_EQ(accs.angular_acc[1], 0); 
    EXPECT_FLOAT_EQ(accs.angular_acc[2], 0);  


    //check time step = dt
    EXPECT_FLOAT_EQ(step_current, 0.02);  
}

TEST_F(rotorTMRigidBodyTest, applyRandomnForceTwoSteps){

    // define radom force and zero torque
    const Eigen::Vector3d force = Eigen::Vector3d::Random();
    //const Eigen::Vector3d force(1, 1, 1);

    const Eigen::Vector3d torque(0, 0, 0);
    
    const Wrench mav_wrench = {force, torque};

    // input force and torque
    // ptr_rigidbody->InputForce(force);
    // ptr_rigidbody->InputTorque(torque);
     ptr_rigidbody->InputWrench(mav_wrench);

    // do two steps integration
    const double dt = 0.01;
    for(double t=dt ; t<=2*dt ; t+= dt)
    {
            ptr_rigidbody->DoOneStepInt();
            // printf("current step is %.3f \n", t);
    }
    

    Pose pose =  ptr_rigidbody->pose();
    Vels vels = ptr_rigidbody->vels();    
    Accs accs = ptr_rigidbody->accs();
    double step_current = ptr_rigidbody->timestep();

    // check position = 0.5at^2

    EXPECT_FLOAT_EQ(pose.post[0], 0.5 * (force[0]) * pow(0.02,2)); 
    EXPECT_FLOAT_EQ(pose.post[1], 0.5 * (force[1]) * pow(0.02,2)); 
    EXPECT_FLOAT_EQ(pose.post[2], 0.5 * (force[2]-9.8) * pow(0.02,2)); 

    // check vel = at
    
    

    EXPECT_FLOAT_EQ(vels.linear_vel[0], (force[0]) *0.02);
    EXPECT_FLOAT_EQ(vels.linear_vel[1], (force[1]) *0.02);
    EXPECT_FLOAT_EQ(vels.linear_vel[2], (force[2]-9.8) *0.02);

    // check acc = a
    
    
    EXPECT_FLOAT_EQ(accs.linear_acc[0], force[0]); 
    EXPECT_FLOAT_EQ(accs.linear_acc[1], force[1]); 
    EXPECT_FLOAT_EQ(accs.linear_acc[2], (force[2]-9.8));  


    // check attitude = 0
    
    
    EXPECT_FLOAT_EQ(pose.att.x(), 0); 
    EXPECT_FLOAT_EQ(pose.att.y(), 0); 
    EXPECT_FLOAT_EQ(pose.att.z(), 0);  
    EXPECT_FLOAT_EQ(pose.att.w(), 1);  

    // check bodyrate =0
    
    
    EXPECT_FLOAT_EQ(vels.bodyrate[0], 0); 
    EXPECT_FLOAT_EQ(vels.bodyrate[1], 0); 
    EXPECT_FLOAT_EQ(vels.bodyrate[2], 0);  

    // check angular_acc =0
    
    
    EXPECT_FLOAT_EQ(accs.angular_acc[0], 0); 
    EXPECT_FLOAT_EQ(accs.angular_acc[1], 0); 
    EXPECT_FLOAT_EQ(accs.angular_acc[2], 0);  

    //check time step = dt
    
    
    EXPECT_FLOAT_EQ(step_current, 0.02);  
}






TEST_F(rotorTMRigidBodyTest, applyRandomnForceTenSteps){

    // define radom force and zero torque
    const Eigen::Vector3d force = Eigen::Vector3d::Random();
    //const Eigen::Vector3d force(1, 1, 1);

    const Eigen::Vector3d torque(0, 0, 0);
    
    const Wrench mav_wrench = {force, torque};

    // input force and torque
    // ptr_rigidbody->InputForce(force);
    // ptr_rigidbody->InputTorque(torque);
    ptr_rigidbody->InputWrench(mav_wrench);

    // do 10 steps integration
    const double dt = 0.01;
    for(double t=dt ; t<=10*dt ; t+= dt)
    {
            ptr_rigidbody->DoOneStepInt();
            // printf("current step is %.3f \n", t);
    }
    
    Pose pose =  ptr_rigidbody->pose();
    Vels vels = ptr_rigidbody->vels();    
    Accs accs = ptr_rigidbody->accs();
    double step_current = ptr_rigidbody->timestep();

    // check position = 0.5at^2
    EXPECT_FLOAT_EQ(pose.post[0], 0.5 * (force[0]) * pow(10*dt,2)); 
    EXPECT_FLOAT_EQ(pose.post[1], 0.5 * (force[1]) * pow(10*dt,2)); 
    EXPECT_FLOAT_EQ(pose.post[2], 0.5 * (force[2]-9.8) * pow(10*dt,2)); 

    // check vel = at
    
    

    EXPECT_FLOAT_EQ(vels.linear_vel[0], (force[0]) *10*dt);
    EXPECT_FLOAT_EQ(vels.linear_vel[1], (force[1]) *10*dt);
    EXPECT_FLOAT_EQ(vels.linear_vel[2], (force[2]-9.8) *10*dt);

    // check acc = a
    
    
    EXPECT_FLOAT_EQ(accs.linear_acc[0], force[0]); 
    EXPECT_FLOAT_EQ(accs.linear_acc[1], force[1]); 
    EXPECT_FLOAT_EQ(accs.linear_acc[2], (force[2]-9.8));  

    // check attitude = 0
    
    
    EXPECT_FLOAT_EQ(pose.att.x(), 0); 
    EXPECT_FLOAT_EQ(pose.att.y(), 0); 
    EXPECT_FLOAT_EQ(pose.att.z(), 0);  
    EXPECT_FLOAT_EQ(pose.att.w(), 1);  

    // check bodyrate =0
    
    
    EXPECT_FLOAT_EQ(vels.bodyrate[0], 0); 
    EXPECT_FLOAT_EQ(vels.bodyrate[1], 0); 
    EXPECT_FLOAT_EQ(vels.bodyrate[2], 0);  

    // check angular_acc =0
    
    
    EXPECT_FLOAT_EQ(accs.angular_acc[0], 0); 
    EXPECT_FLOAT_EQ(accs.angular_acc[1], 0); 
    EXPECT_FLOAT_EQ(accs.angular_acc[2], 0);  

    //check time step = dt
    
    
    ASSERT_FLOAT_EQ(step_current, 10*dt);  
}

TEST_F(rotorTMRigidBodyTest, applyRandomnForceThousandSteps){

    // define radom force and zero torque
    const Eigen::Vector3d force = Eigen::Vector3d::Random();
    //const Eigen::Vector3d force(1, 1, 1);

    const Eigen::Vector3d torque(0, 0, 0);
    
    const Wrench mav_wrench = {force, torque};

    // input force and torque
    // ptr_rigidbody->InputForce(force);
    // ptr_rigidbody->InputTorque(torque);
    ptr_rigidbody->InputWrench(mav_wrench);

    // do two steps integration
    const double dt = 0.01;
    for(double t=dt ; t<=1000*dt ; t+= dt)
    {
            ptr_rigidbody->DoOneStepInt();
            // printf("current step is %.3f \n", t);
    }
    

    Pose pose =  ptr_rigidbody->pose();
    Vels vels = ptr_rigidbody->vels();    
    Accs accs = ptr_rigidbody->accs();
    double step_current = ptr_rigidbody->timestep();


    // check position = 0.5at^2
    EXPECT_FLOAT_EQ(pose.post[0], 0.5 * (force[0]) * pow(1000*dt,2)); 
    EXPECT_FLOAT_EQ(pose.post[1], 0.5 * (force[1]) * pow(1000*dt,2)); 
    EXPECT_FLOAT_EQ(pose.post[2], 0.5 * (force[2]-9.8) * pow(1000*dt,2)); 

    // check vel = at
    
    

    EXPECT_FLOAT_EQ(vels.linear_vel[0], (force[0]) *1000*dt);
    EXPECT_FLOAT_EQ(vels.linear_vel[1], (force[1]) *1000*dt);
    EXPECT_FLOAT_EQ(vels.linear_vel[2], (force[2]-9.8) *1000*dt);

    // check acc = a
    
    
    EXPECT_FLOAT_EQ(accs.linear_acc[0], force[0]); 
    EXPECT_FLOAT_EQ(accs.linear_acc[1], force[1]); 
    EXPECT_FLOAT_EQ(accs.linear_acc[2], (force[2]-9.8));  


    // check attitude = 0
    
    
    EXPECT_FLOAT_EQ(pose.att.x(), 0); 
    EXPECT_FLOAT_EQ(pose.att.y(), 0); 
    EXPECT_FLOAT_EQ(pose.att.z(), 0);  
    EXPECT_FLOAT_EQ(pose.att.w(), 1);  

    // check bodyrate =0
    
    
    EXPECT_FLOAT_EQ(vels.bodyrate[0], 0); 
    EXPECT_FLOAT_EQ(vels.bodyrate[1], 0); 
    EXPECT_FLOAT_EQ(vels.bodyrate[2], 0);  

    // check angular_acc =0
    
    
    EXPECT_FLOAT_EQ(accs.angular_acc[0], 0); 
    EXPECT_FLOAT_EQ(accs.angular_acc[1], 0); 
    EXPECT_FLOAT_EQ(accs.angular_acc[2], 0);  


    //check time step = dt
    
    
    ASSERT_FLOAT_EQ(step_current, 1000*dt);  
}

TEST_F(rotorTMRigidBodyTest, applyRandomnForceNonOriginThousandSteps){

    // define radom force and zero torque
    const Eigen::Vector3d force = Eigen::Vector3d::Random();
    //const Eigen::Vector3d force(1, 1, 1);

    const Eigen::Vector3d torque(0, 0, 0);
    
    // initial position of quadrotor
    const Eigen::Vector3d initial_post = Eigen::Vector3d::Random();
    ptr_rigidbody->SetInitialPost(initial_post);


    const Wrench mav_wrench = {force, torque};

    // input force and torque
    // ptr_rigidbody->InputForce(force);
    // ptr_rigidbody->InputTorque(torque);
    ptr_rigidbody->InputWrench(mav_wrench);

    // do two steps integration
    const double dt = 0.01;
    for(double t=dt ; t<=1000*dt ; t+= dt)
    {
            ptr_rigidbody->DoOneStepInt();
            // printf("current step is %.3f \n", t);
    }
    

    Pose pose =  ptr_rigidbody->pose();
    Vels vels = ptr_rigidbody->vels();    
    Accs accs = ptr_rigidbody->accs();
    double step_current = ptr_rigidbody->timestep();


    // check position = 0.5at^2
    

    

    EXPECT_FLOAT_EQ(pose.post[0], initial_post[0]+0.5 * (force[0]) * pow(1000*dt,2)); 
    EXPECT_FLOAT_EQ(pose.post[1], initial_post[1]+0.5 * (force[1]) * pow(1000*dt,2)); 
    EXPECT_FLOAT_EQ(pose.post[2], initial_post[2]+0.5 * (force[2]-9.8) * pow(1000*dt,2)); 

    // check vel = at
    
    

    EXPECT_FLOAT_EQ(vels.linear_vel[0], (force[0]) *1000*dt);
    EXPECT_FLOAT_EQ(vels.linear_vel[1], (force[1]) *1000*dt);
    EXPECT_FLOAT_EQ(vels.linear_vel[2], (force[2]-9.8) *1000*dt);

    // check acc = a
    
    
    EXPECT_FLOAT_EQ(accs.linear_acc[0], force[0]); 
    EXPECT_FLOAT_EQ(accs.linear_acc[1], force[1]); 
    EXPECT_FLOAT_EQ(accs.linear_acc[2], (force[2]-9.8));  


    // check attitude = 0
    
    
    EXPECT_FLOAT_EQ(pose.att.x(), 0); 
    EXPECT_FLOAT_EQ(pose.att.y(), 0); 
    EXPECT_FLOAT_EQ(pose.att.z(), 0);  
    EXPECT_FLOAT_EQ(pose.att.w(), 1);  

    // check bodyrate =0
    
    
    EXPECT_FLOAT_EQ(vels.bodyrate[0], 0); 
    EXPECT_FLOAT_EQ(vels.bodyrate[1], 0); 
    EXPECT_FLOAT_EQ(vels.bodyrate[2], 0);  

    // check angular_acc =0
    
    
    EXPECT_FLOAT_EQ(accs.angular_acc[0], 0); 
    EXPECT_FLOAT_EQ(accs.angular_acc[1], 0); 
    EXPECT_FLOAT_EQ(accs.angular_acc[2], 0);  


    //check time step = dt
    
    
    ASSERT_FLOAT_EQ(step_current, 1000*dt);  
}

TEST_F(rotorTMRigidBodyTest, applyTorque4XRotation){

    // define force to compensite gravity and zero torque
    const Eigen::Vector3d force(0, 0, 9.8);

    const Eigen::Vector3d torque(4.5, 0, 0);
    
    const Wrench mav_wrench = {force, torque};

    // input force and torque
    // ptr_rigidbody->InputForce(force);
    // ptr_rigidbody->InputTorque(torque);
    ptr_rigidbody->InputWrench(mav_wrench);

    // do one integration
    ptr_rigidbody->DoOneStepInt();



    Pose pose =  ptr_rigidbody->pose();
    Vels vels = ptr_rigidbody->vels();    
    Accs accs = ptr_rigidbody->accs();
    double step_current = ptr_rigidbody->timestep();

    // check position = 0.5at^2
    EXPECT_FLOAT_EQ(pose.post[0], 0); 
    EXPECT_FLOAT_EQ(pose.post[1], 0); 
    EXPECT_FLOAT_EQ(pose.post[2], 0);   

    // check vel = at
    
    

    EXPECT_FLOAT_EQ(vels.linear_vel[0], 0); 
    EXPECT_FLOAT_EQ(vels.linear_vel[1], 0); 
    EXPECT_FLOAT_EQ(vels.linear_vel[2], 0);  

    // check acc = 0
    
    
    EXPECT_FLOAT_EQ(accs.linear_acc[0], 0); 
    EXPECT_FLOAT_EQ(accs.linear_acc[1], 0); 
    EXPECT_FLOAT_EQ(accs.linear_acc[2], 0);  

    // check bodyrate
    
    
    EXPECT_FLOAT_EQ(vels.bodyrate[0], 0.045); 
    EXPECT_FLOAT_EQ(vels.bodyrate[1], 0); 
    EXPECT_FLOAT_EQ(vels.bodyrate[2], 0);  

    // check attitude
    
    
    EXPECT_FLOAT_EQ(pose.att.x(), 0.00011249999976269531); 
    EXPECT_FLOAT_EQ(pose.att.y(), 0); 
    EXPECT_FLOAT_EQ(pose.att.z(), 0);  
    EXPECT_FLOAT_EQ(pose.att.w(), 0.99999999367187498);  

    // check angular_acc
    
    
    EXPECT_FLOAT_EQ(accs.angular_acc[0], 4.5); 
    EXPECT_FLOAT_EQ(accs.angular_acc[1], 0); 
    EXPECT_FLOAT_EQ(accs.angular_acc[2], 0);  


    //check time step = dt
    
    
    EXPECT_FLOAT_EQ(step_current, 0.01);  
}

TEST_F(rotorTMRigidBodyTest, applyTorque4YRotation){

    // define force to compensite gravity and zero torque
    const Eigen::Vector3d force(0, 0, 9.8);

    const Eigen::Vector3d torque(0, 5.25, 0);
    
    const Wrench mav_wrench = {force, torque};

    // input force and torque
    // ptr_rigidbody->InputForce(force);
    // ptr_rigidbody->InputTorque(torque);
    ptr_rigidbody->InputWrench(mav_wrench);

    // do one integration
    ptr_rigidbody->DoOneStepInt();



    Pose pose =  ptr_rigidbody->pose();
    Vels vels = ptr_rigidbody->vels();    
    Accs accs = ptr_rigidbody->accs();
    double step_current = ptr_rigidbody->timestep();


    // check position = 0.5at^2
    

    

    EXPECT_FLOAT_EQ(pose.post[0], 0); 
    EXPECT_FLOAT_EQ(pose.post[1], 0); 
    EXPECT_FLOAT_EQ(pose.post[2], 0);   

    // check vel = at
    
    

    EXPECT_FLOAT_EQ(vels.linear_vel[0], 0); 
    EXPECT_FLOAT_EQ(vels.linear_vel[1], 0); 
    EXPECT_FLOAT_EQ(vels.linear_vel[2], 0);  

    // check acc = 0
    
    
    EXPECT_FLOAT_EQ(accs.linear_acc[0], 0); 
    EXPECT_FLOAT_EQ(accs.linear_acc[1], 0); 
    EXPECT_FLOAT_EQ(accs.linear_acc[2], 0);  


    // check bodyrate
    
    
    EXPECT_FLOAT_EQ(vels.bodyrate[0], 0); 
    EXPECT_FLOAT_EQ(vels.bodyrate[1], 0.0525); 
    EXPECT_FLOAT_EQ(vels.bodyrate[2], 0);  

    // check attitude
    
    
    EXPECT_FLOAT_EQ(pose.att.x(), 0); 
    EXPECT_FLOAT_EQ(pose.att.y(), 0.00013124999962316894); 
    EXPECT_FLOAT_EQ(pose.att.z(), 0);  
    EXPECT_FLOAT_EQ(pose.att.w(), 0.99999999138671880);  

    // check angular_acc
    
    
    EXPECT_FLOAT_EQ(accs.angular_acc[0], 0); 
    EXPECT_FLOAT_EQ(accs.angular_acc[1], 5.25); 
    EXPECT_FLOAT_EQ(accs.angular_acc[2], 0);  


    //check time step = dt
    
    
    EXPECT_FLOAT_EQ(step_current, 0.01);  
}


TEST_F(rotorTMRigidBodyTest, applyTorque4ZRotation){

    // define force to compensite gravity and zero torque
    const Eigen::Vector3d force(0, 0, 9.8);

    const Eigen::Vector3d torque(0, 0, 6.13);
    
    const Wrench mav_wrench = {force, torque};

    // input force and torque
    // ptr_rigidbody->InputForce(force);
    // ptr_rigidbody->InputTorque(torque);
    ptr_rigidbody->InputWrench(mav_wrench);

    // do one integration
    ptr_rigidbody->DoOneStepInt();



    Pose pose =  ptr_rigidbody->pose();
    Vels vels = ptr_rigidbody->vels();    
    Accs accs = ptr_rigidbody->accs();
    double step_current = ptr_rigidbody->timestep();


    // check position = 0.5at^2
    

    

    EXPECT_FLOAT_EQ(pose.post[0], 0); 
    EXPECT_FLOAT_EQ(pose.post[1], 0); 
    EXPECT_FLOAT_EQ(pose.post[2], 0);   

    // check vel = at
    
    

    EXPECT_FLOAT_EQ(vels.linear_vel[0], 0); 
    EXPECT_FLOAT_EQ(vels.linear_vel[1], 0); 
    EXPECT_FLOAT_EQ(vels.linear_vel[2], 0);  

    // check acc = 0
    
    
    EXPECT_FLOAT_EQ(accs.linear_acc[0], 0); 
    EXPECT_FLOAT_EQ(accs.linear_acc[1], 0); 
    EXPECT_FLOAT_EQ(accs.linear_acc[2], 0);  

    // check bodyrate
    
    
    EXPECT_FLOAT_EQ(vels.bodyrate[0], 0); 
    EXPECT_FLOAT_EQ(vels.bodyrate[1], 0); 
    EXPECT_FLOAT_EQ(vels.bodyrate[2],  0.0613);  

    // check attitude
    
    
    EXPECT_FLOAT_EQ(pose.att.x(), 0); 
    EXPECT_FLOAT_EQ(pose.att.y(), 0); 
    EXPECT_FLOAT_EQ(pose.att.z(), 0.0001532499994001396);  
    EXPECT_FLOAT_EQ(pose.att.w(), 0.9999999882572188);  

    // check angular_acc
    
    
    EXPECT_FLOAT_EQ(accs.angular_acc[0], 0); 
    EXPECT_FLOAT_EQ(accs.angular_acc[1], 0); 
    EXPECT_FLOAT_EQ(accs.angular_acc[2], 6.13);  


    //check time step = dt
    
    
    EXPECT_FLOAT_EQ(step_current, 0.01);  
}



TEST_F(rotorTMRigidBodyTest, applyTorque4XYZRotationTenSteps){

    // define force to compensite gravity and zero torque
    const Eigen::Vector3d force(0, 0, 9.8);

    const Eigen::Vector3d torque(0.1, 0.2, 0.3);
    
    const Wrench mav_wrench = {force, torque};

    // input force and torque
    // ptr_rigidbody->InputForce(force);
    // ptr_rigidbody->InputTorque(torque);
    ptr_rigidbody->InputWrench(mav_wrench);

    // do 10 steps integration
    const double dt = 0.01;
    for(double t=dt ; t<=10*dt ; t+= dt)
    {
            ptr_rigidbody->DoOneStepInt();
            // printf("current step is %.3f \n", t);
    }
    

    Pose pose =  ptr_rigidbody->pose();
    Vels vels = ptr_rigidbody->vels();    
    Accs accs = ptr_rigidbody->accs();
    double step_current = ptr_rigidbody->timestep();



    // check position = 0.5at^2
    

    

    EXPECT_FLOAT_EQ(pose.post[0], 0); 
    EXPECT_FLOAT_EQ(pose.post[1], 0); 
    EXPECT_FLOAT_EQ(pose.post[2], 0);   

    // check vel = at
    
    

    EXPECT_FLOAT_EQ(vels.linear_vel[0], 0); 
    EXPECT_FLOAT_EQ(vels.linear_vel[1], 0); 
    EXPECT_FLOAT_EQ(vels.linear_vel[2], 0);  

    //check acc = 0
    
    
    EXPECT_FLOAT_EQ(accs.linear_acc[0], 0); 
    EXPECT_FLOAT_EQ(accs.linear_acc[1], 0); 
    EXPECT_FLOAT_EQ(accs.linear_acc[2], 0);  

    // check bodyrate
    
    
    EXPECT_FLOAT_EQ(vels.bodyrate[0], 0.01); 
    EXPECT_FLOAT_EQ(vels.bodyrate[1], 0.02); 
    EXPECT_FLOAT_EQ(vels.bodyrate[2], 0.03);  

    // check attitude
    
    
    EXPECT_NEAR(pose.att.x(), 2.5004037205422324e-4, 1e-5);     
    EXPECT_NEAR(pose.att.y(), 5.0002658173730924e-4, 1e-5);      
    EXPECT_NEAR(pose.att.z(), 7.5006364505842392e-4, 1e-5);     
    EXPECT_NEAR(pose.att.w(), 9.9999956242878352e-1, 1e-5);      

    // check angular_acc
    
    
    EXPECT_FLOAT_EQ(accs.angular_acc[0], 0.1); 
    EXPECT_FLOAT_EQ(accs.angular_acc[1], 0.2); 
    EXPECT_FLOAT_EQ(accs.angular_acc[2], 0.3);  


    //check time step = dt
    
    
    ASSERT_FLOAT_EQ(step_current, 10*dt);  
}

TEST_F(rotorTMRigidBodyTest, applyTorque4XYZRotationThreeHundSteps){

    // define force to compensite gravity and zero torque
    const Eigen::Vector3d force(0, 0, 9.8);

    const Eigen::Vector3d torque(0.1, 0.2, 0.3);
    
    const Wrench mav_wrench = {force, torque};

    // input force and torque
    // ptr_rigidbody->InputForce(force);
    // ptr_rigidbody->InputTorque(torque);
    ptr_rigidbody->InputWrench(mav_wrench);

    // do 10 steps integration
    const double dt = 0.01;
    for(double t=dt ; t<=300*dt ; t+= dt)
    {
            ptr_rigidbody->DoOneStepInt();
            // printf("current step is %.3f \n", t);
    }
    

    Pose pose =  ptr_rigidbody->pose();
    Vels vels = ptr_rigidbody->vels();    
    Accs accs = ptr_rigidbody->accs();
    double step_current = ptr_rigidbody->timestep();



    // check position = 0.5at^2
    

    

    EXPECT_FLOAT_EQ(pose.post[0], 0); 
    EXPECT_FLOAT_EQ(pose.post[1], 0); 
    EXPECT_FLOAT_EQ(pose.post[2], 0);   

    // check vel = at
    
    

    EXPECT_FLOAT_EQ(vels.linear_vel[0], 0); 
    EXPECT_FLOAT_EQ(vels.linear_vel[1], 0); 
    EXPECT_FLOAT_EQ(vels.linear_vel[2], 0);  

    // check acc = 0
    
    
    EXPECT_FLOAT_EQ(accs.linear_acc[0], 0); 
    EXPECT_FLOAT_EQ(accs.linear_acc[1], 0); 
    EXPECT_FLOAT_EQ(accs.linear_acc[2], 0);  

    // check bodyrate
    
    
    EXPECT_FLOAT_EQ(vels.bodyrate[0], 0.3); 
    EXPECT_FLOAT_EQ(vels.bodyrate[1], 0.6); 
    EXPECT_FLOAT_EQ(vels.bodyrate[2], 0.9);  

    // check attitude
    
    
    EXPECT_NEAR(pose.att.x(), 0.19934871398234738, 1e-5);    
    EXPECT_NEAR(pose.att.y(), 0.3986953792988834, 1e-5);  
    EXPECT_NEAR(pose.att.z(), 0.5980442862305684, 1e-5);  
    EXPECT_NEAR(pose.att.w(), 0.6660669008938018, 1e-5);  

    // check angular_acc
    
    
    EXPECT_FLOAT_EQ(accs.angular_acc[0], 0.1); 
    EXPECT_FLOAT_EQ(accs.angular_acc[1], 0.2); 
    EXPECT_FLOAT_EQ(accs.angular_acc[2], 0.3);  
    
    //check time step = dt
    
    
    ASSERT_FLOAT_EQ(step_current, 300*dt);  
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
