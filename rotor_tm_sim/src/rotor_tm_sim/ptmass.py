 # Second Scenario: Point Mass
                if self.nquad == 1:
                    
                    ## first check for inelastic collision
                    pl_pos = x[0:3]
                    pl_vel = x[3:6]
                    robot_pos = x[13:16]
                    robot_vel = x[16:19]
                    cable_norm_vel = np.transpose(pl_pos - robot_pos) @ (pl_vel - robot_vel)/np.linalg.norm(pl_pos - robot_pos)
                    print("cable norm vel is", cable_norm_vel, "\n",sep="---")
                    ## if collision, compute new velocities and assign to state
                    if cable_norm_vel > 1e-3 and not self.cable_is_slack:
                        print("Colliding")
                        v1, v2 = self.ptmass_inelastic_cable_collision(x[0:6], x[13:19], self.pl_params.mass, self.uav_params[0].mass)
                        x[3:6] = v1
                        x[16:19] = v2
                    
                    ## set up event for ivp solver
                    ptmasstautToSlack.terminal = True
                    ptmassslackToTaut.terminal = True
                    ptmasstautToSlack.direction = -1
                    ptmassslackToTaut.direction = 1
                    ptmassslackToTaut.cable_length = self.pl_params.cable_length
                    ptmasstautToSlack.cable_length = self.pl_params.cable_length

                    ## state integration
                    if self.cable_is_slack:
                        print("Cable is slack")
                        #print(x)
                        sol = scipy.integrate.solve_ivp(self.hybrid_ptmass_pl_transportationEOM, t_span, x, method= 'RK45', t_eval=t_span, events=ptmassslackToTaut)
                    else:
                        print("Cable is taut")
                        #print(x)
                        sol = scipy.integrate.solve_ivp(self.hybrid_ptmass_pl_transportationEOM, t_span, x, method= 'RK45', t_eval=t_span, events=ptmasstautToSlack)
                    
                    ## extract state from solver soltion
                    if (np.all(x==sol.y[:, -1])) and (len(sol.y_events[0]) != 0):
                        x = sol.y_events[0][:]
                        x = x.T
                        x = x.reshape((x.shape[0],))
                    else:
                        x = sol.y[:,-1]
                    
                    ## recheck cable slack condition
                    self.cable_is_slack = self.isslack(x[0:3], x[13:16], self.pl_params.cable_length)

   def hybrid_ptmass_pl_transportationEOM(self, t, s):
    # DESCRIPTION:
    # QUADEOM Wrapper function for solving quadrotor equation of motion
    # 	quadEOM takes in time, state vector, and output the derivative of the state vector, the
    # 	actual calcution is done in quadEOM_readonly.
    #
    # INPUTS:
    # t             - 1 x 1, time
    # s computed    - 19 x 1, state vector = [xL, yL, zL, xLd, yLd, zLd,
    #                                         xQ, yQ, zQ, xQd, yQd, zQd,
    #                                         qw, qx, qy, qz, pQ, qQ, rQ]
    #
    # take_in/output- 26 x 1, state vector(dot-elementwise) = [ xL,     yL,     zL,     xLd,    yLd,    zLd,
    #                                                           qLw = 0,qLx = 0,qLy = 0,qLz = 0,pL = 0, qL = 0, rL = 0,
    #                                                           xQ,     yQ,     zQ,     xQd,    yQd,    zQd
    #                                                           qw,     qx,     qy,     qz,     pQuad,  qQuad,  rQuad]
    #
    #                   ,where [xL, yL, zL] is payload postion
    #                          [xLd, yLd, zLd], is payload linear velocity
    #                          [xQ, yQ, zQ] is quadrotor position
    #                          [xQd, yQd, zQd] is quadrotor velocity
    #                          [qw, qx, qy, qz] is quadrotor orientation in quaternion
    #                          [pQ, qQ, rQ] is quadrotor angular velocity in its own frame
    # OUTPUTS:
    # sdot          - 26 x 1, derivative of state vector s as mentioned above (some values are forced to 0)
      l = self.pl_params.cable_length
      plqd = {}
      # convert state s to plqd
      plqd["pos"] = s[0:3]
      plqd["vel"] = s[3:6]
      plqd["qd_pos"] = s[13:16]
      plqd["qd_vel"] = s[16:19]
      plqd["qd_quat"] = s[19:23]
      plqd["qd_omega"] = s[23:26]
      Rot = utilslib.QuatToRot(s[19:23])
      plqd["qd_rot"] = Rot
      quad_load_rel_pos = plqd["qd_pos"]-plqd["pos"]
      quad_load_rel_vel = plqd["qd_vel"]-plqd["vel"]

      if self.cable_is_slack[0]:   
          return self.slack_ptmass_payload_quadEOM_readonly(t, plqd, self.uav_F, self.uav_M)
      else:
          plqd["xi"] = -quad_load_rel_pos/l
          plqd["xidot"] = -quad_load_rel_vel/l
          return self.taut_ptmass_payload_quadEOM_readonly(t, plqd, self.uav_F, self.uav_M)

  def slack_ptmass_payload_quadEOM_readonly(self, t, plqd, F, M):
      # DESCRIPTION:
      # slack_ptmass_payload_quadEOM_readonly Solve payload equation of motion
      # calculating the derivative of the state vector when cable is slack
          
      # INPUTS:
      # plqd - struct, quadrotor and payload parameters you want to pass in
      # F - ndarray, thrust value from controller, in quadrotor frame
      # M - ndarray, control moment from controller, in quadrotor frame
      # t - float, time, not used currently
          
      # OUTPUTS:
      # sdot   - 26 x 1, derivative of state vector s
      # # Assign params and states
      mQ  =   self.uav_params[0].mass
      e3  =   np.array([[0.0],[0.0],[1.0]])
      g   =   self.uav_params[0].grav * e3
      wRb =   plqd["qd_rot"];   # Rotation matrix of the quadrotor
      qd_quat     =   plqd["qd_quat"]
      qd_omega    =   plqd["qd_omega"]
      p = qd_omega[0]
      q = qd_omega[1]
      r = qd_omega[2]

      # Obtain Quadrotor Force Vector
      quad_force_vector = F * wRb @ e3 

      # Solving for Quadrotor Acceleration
      accQ = quad_force_vector/mQ - g; 

      # Solving for Quadrotor Angular Velocity
      K_quat = 2.0 # this enforces the magnitude 1 constraint for the quaternion
      quaterror = 1 - np.linalg.norm(qd_quat)
      qdot =  1/2*np.array([[0, -p, -q, -r],
                            [p,  0,  r, -q],
                            [q, -r,  0,  p],
                            [r,  q, -p,  0]]) @ qd_quat.reshape((qd_quat.shape[0], 1)) + K_quat * quaterror * qd_quat.reshape((qd_quat.shape[0], 1))

      # Solving for Quadrotor Angular Acceleration
      pqrdot   = self.uav_params[0].invI @ (M - np.reshape(np.cross(qd_omega, self.uav_params[0].I @ qd_omega, axisa=0, axisb=0), (3,1)))
      
      # Assemble sdot
      sdot = np.zeros((26,1), dtype=float)
      sdot[0:3] = plqd["vel"].reshape(3, 1)
      sdot[3:6] = -g
      sdot[13:16] = plqd["qd_vel"].reshape(3, 1)
      sdot[16:19] = accQ
      sdot[19:23] = qdot
      sdot[23:26] = pqrdot
      sdot = sdot[:,0]
      return sdot

  def taut_ptmass_payload_quadEOM_readonly(self, t, plqd, F, M):
      # DESCRIPTION:
      # taut_ptmass_payload_quadEOM_readonly Solve payload equation of motion
      # calculating the derivative of the state vector when cable is taut
          
      # INPUTS:
      # plqd - struct, quadrotor and payload parameters you want to pass in
      # F - ndarray, thrust value from controller, in quadrotor frame
      # M - ndarray, control moment from controller, in quadrotor frame
      # t - float, time, not used currently
          
      # OUTPUTS:
      # sdot   - 26 x 1, derivative of state vector s
      mL = self.pl_params.mass
      mQ = self.uav_params[0].mass
      total_mass = mL + mQ
      l = self.pl_params.cable_length
      xi = plqd["xi"]
      xidot = plqd["xidot"]
      xi_omega = np.cross(xi,xidot)
      e3=np.array([[0.0],[0.0],[1.0]])
      g = self.uav_params[0].grav * e3
      wRb=plqd["qd_rot"]    #Rotation matrix of the quadrotor
      qd_quat = plqd["qd_quat"]
      qd_omega = plqd["qd_omega"]
      p = qd_omega[0]
      q = qd_omega[1]
      r = qd_omega[2]

      # Obtain tension vector
      quad_force_vector = F * wRb @ e3
      quad_centrifugal_f = mQ * l * (xi_omega.T @ xi_omega)
      tension_vector = mL * (-xi.T.reshape(1,3) @ quad_force_vector + quad_centrifugal_f) * xi.reshape(3,1) / total_mass
      # Solving for Load Acceleration
      accL = - tension_vector / mL - g
      # Solving for Quadrotor Acceleration
      accQ = (quad_force_vector + tension_vector) / mQ - g

      # Solving for Quadrotor Angular Velocity
      K_quat = 2 # this enforces the magnitude 1 constraint for the quaternion
      quaterror = 1 - np.linalg.norm(qd_quat)
      qdot =  1/2*np.array([[0, -p, -q, -r],
                            [p,  0,  r, -q],
                            [q, -r,  0,  p],
                            [r,  q, -p,  0]]) @ qd_quat.reshape((qd_quat.shape[0], 1)) + K_quat * quaterror * qd_quat.reshape((qd_quat.shape[0], 1))

      # Solving for Quadrotor Angular Acceleration
      pqrdot   = self.uav_params[0].invI @ (M - np.cross(qd_omega, self.uav_params[0].I @ qd_omega, axisa=0, axisb=0).T.reshape(3, 1))

      # Assemble sdot
      sdot = np.zeros((26,1), dtype=float)
      sdot[0:3] = plqd["vel"].reshape(3, 1)
      sdot[3:6] = accL
      sdot[13:16] = plqd["qd_vel"].reshape(3, 1)
      sdot[16:19] = accQ
      sdot[19:23] = qdot
      sdot[23:26] = pqrdot
      sdot = sdot[:,0]

      return sdot

  def ptmass_inelastic_cable_collision(self, x1, x2, m1, m2): 
      # DESCRIPTION:
      # ptmass_inelastic_cable_collision 
      # redistribute the velocity between payload and the quadrotor by assuming
      # perfectly inelastic collision of ptmass and the drone along the cable direction
          
      # INPUTS:
      # x1 - float, position of object 1
      # x2 - float, position of object 2
      # m1 - float, mass of object 1
      # m2 - float, mass of object 2
          
      # OUTPUTS:
      # v1, v2   - both float, new velocities for the two object
      
      obj1_pos = x1[0:3]
      obj1_pos = obj1_pos.reshape((3, 1))
      obj2_pos = x2[0:3]
      obj2_pos = obj2_pos.reshape((3, 1))
      obj1_vel = x1[3:6]
      obj2_pos = obj2_pos.reshape((3, 1))
      obj2_vel = x2[3:6]
      obj2_pos = obj2_pos.reshape((3, 1))

      cable_direction = (obj2_pos - obj1_pos) / np.linalg.norm(obj2_pos - obj1_pos)
      cable_direction = cable_direction.reshape((3, 1))
      cable_direction_projmat = cable_direction @ cable_direction.T
      rospy.loginfo(cable_direction_projmat)
      print(cable_direction_projmat)
      v1_proj = cable_direction_projmat @ obj1_vel
      v2_proj = cable_direction_projmat @ obj2_vel
      
      v = (m1 * v1_proj + m2 * v2_proj)/(m1+m2)
      
      v1 = v + obj1_vel - v1_proj
      v2 = v + obj2_vel - v2_proj

      return v1, v2


def ptmassslackToTaut(t, x):
    # DESCRIPTION:
    # event function for point mass scenario dynammics 
    # if event is reached by ivp solver, it will switch from slack to taut, takes in t (time) and state (x)

    # INPUTS:
    # t             - time
    # x             - state of the point mass system. Specifically,
    #                 x is a 26 by 1 ndarray,
    #                                                               Name                 Last Element Location (counting from 1) 
    #                 x = np.array([ppx,  ppy,    ppz,            # payload position    3
    #                               pvx,  pvy,    pvz,            # payload velocity    6
    #                                pu,   pi,     pj,     pk,    # payload quat        10
    #                               pwx,  pwy,    pwz,            # payload omega       13
    #                               qpx,  qpy,    qpz,            # quad rotor position 16
    #                               qvx,  qvy,    qvz,            # quad rotor velocity 19 
    #                                qu,   qi,     qj,     qk,    # quad rotor quat     23
    #                               qwx,  qwy,    qwz])           # quad rotor omega    26

    # OUTPUTS:
    # value         - a float that determines taut condition
    value = np.linalg.norm(x[0:3] - x[13:16]) - ptmassslackToTaut.cable_length
    return value

def ptmasstautToSlack(t, x):
    # DESCRIPTION:
    # event function for point mass scenario dynammics 
    # if event is reached by ivp solver, it will switch from taut to slack, takes in t (time) and state (x)

    # INPUTS:
    # t             - time
    # x             - state of the point mass system. Specifically,
    #                 x is a 26 by 1 ndarray,
    #                                                               Name                 Last Element Location (counting from 1) 
    #                 x = np.array([ppx,  ppy,    ppz,            # payload position    3
    #                               pvx,  pvy,    pvz,            # payload velocity    6
    #                                pu,   pi,     pj,     pk,    # payload quat        10
    #                               pwx,  pwy,    pwz,            # payload omega       13
    #                               qpx,  qpy,    qpz,            # quad rotor position 16
    #                               qvx,  qvy,    qvz,            # quad rotor velocity 19 
    #                                qu,   qi,     qj,     qk,    # quad rotor quat     23
    #                               qwx,  qwy,    qwz])           # quad rotor omega    26

    # OUTPUTS:
    # value         - a float that determines slack condition
    value = np.linalg.norm(x[0:3] - x[13:16]) - ptmasstautToSlack.cable_length + 0.000001
    return value

  def isslack(self, robot_pos, attach_pos, cable_length):
    # DESCRIPTION:
    # This method takes in quad rotor position, cable attchment position
    # and cable and determines if the cable is slack or not.
    # This method is only used in for point mass case
    
    # INPUTS:
    # robot_pos                 - a [3 by 1] ndarray, describing the 3D position
    #                             of the MAV position 
    # attach_pos                - a [3 by 1] ndarray, describing the 3D position
    #                             of the cable attachment position
    # cable_length              - a ndarray (shape of (1,)), the n-th element describes the length
    #                             of the cable used by the n-th robot

    # OUTPUTS:
    # flag                      - an ndarray (shape of (1,)) with boolean type elements
    #                             the element describes if the cable is slack
      if (np.linalg.norm(robot_pos - attach_pos) > (cable_length - 1e-3)):
        return np.array([0.0])
      else:
        return np.array([1.0])
