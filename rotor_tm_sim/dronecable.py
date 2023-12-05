 ## first, we check for collision
                    inelastic_collision_flag, xi = self.cooperative_check_inelastic(x)
                    
                    ## make sure velocities are distributed with no new collisions happening 
                    while np.any(inelastic_collision_flag):
                        print("collision!")
                        before_collide_inelastic_collision_flag = inelastic_collision_flag
                        print("The before inelastic collision_flag is", before_collide_inelastic_collision_flag)
                        x = self.rigidbody_quad_inelastic_cable_collision(x, inelastic_collision_flag)
                        print("collision finished!")
                        after_collide_inelastic_collision_flag, xi = self.cooperative_check_inelastic(x)
                        print("Checking after collision")
                        print("The after inelastic collision_flag is", after_collide_inelastic_collision_flag)
                        if np.any((after_collide_inelastic_collision_flag - before_collide_inelastic_collision_flag)>0):
                            inelastic_collision_flag = after_collide_inelastic_collision_flag + before_collide_inelastic_collision_flag
                            for i in range(inelastic_collision_flag.shape[0]):
                                if inelastic_collision_flag[i] != 0:
                                    inelastic_collision_flag[i] = 1.0
                        else:
                            print("Seems to be last time of the collision")
                            attach_point_position = x[0:3].reshape((3,1))+ utilslib.QuatToRot(x[6:10]) @ self.pl_params.rho_vec_list
                            inelastic_collision_flag = after_collide_inelastic_collision_flag
                            for i in range(self.nquad):
                                print("The robot ", i+1, " position is ", x[(i+1)*13:(i+1)*13+3])
                                print("The xi ", i+1, " is ", xi[:,i])
                                print("The new robot position is, ", attach_point_position[:,i] - self.pl_params.cable_length[i] * xi[:,i])
                                x[(i+1)*13:(i+1)*13+3] = attach_point_position[:,i] - self.pl_params.cable_length[i] * xi[:,i]
                    
                    #print("                                              ")
                    ## set up event for ivp solver
                    slack_condition = self.cable_is_slack
                    idx = np.arange(1, self.pl_params.nquad+1)
                    slack_cable_idx = idx[slack_condition == 1]
                    taut_cable_idx = idx[slack_condition == 0]
                    num_of_slack_cable = np.max(slack_cable_idx.shape)
                    num_of_taut_cable = np.max(taut_cable_idx.shape)

                    GuardEvents = []
                    for i in range(self.nquad):
                        GuardEvents.append(lambda t, x: cooperativeGuard(t, x, GuardEvents[i].pl_params.nquad, GuardEvents[i].slack_condition, GuardEvents[i].pl_params.rho_vec_list, GuardEvents[i].pl_params.cable_length, GuardEvents[i].i))

                    temp = np.zeros((slack_condition.shape), dtype=float)
                    for i in range(self.nquad):
                        if slack_condition[i]:
                            temp[i] = 1.0
                        else:
                            temp[i] = -1.0
                    id = 0
                    for fcn in GuardEvents:
                        fcn.terminal = True
                        if num_of_slack_cable == 0:
                            fcn.direction = -1.0
                        elif num_of_taut_cable == 0:
                            fcn.direction = 1.0
                        else:
                            fcn.direction = temp[id]
                        fcn.pl_params = self.pl_params
                        fcn.slack_condition = slack_condition
                        fcn.i = id
                        id = id + 1

                    ## state integration
                    sol = scipy.integrate.solve_ivp(self.hybrid_cooperative_rigidbody_pl_transportationEOM, t_span, x, method='RK23', t_eval=t_span, events=GuardEvents)
                    
                    ## extract state from solver soltion
                    EventTriggered_bool = sol.status
                    EventTriggered_id = 0

                    if EventTriggered_bool == 1:
                        for i in range(self.nquad):
                            if len(sol.y_events[i]) != 0: 
                                EventTriggered_id = i
                        if (np.all(x==sol.y[:, -1])):
                            x = sol.y_events[EventTriggered_id][:]
                            x = x.T
                            x = x.reshape((x.shape[0],))
                    else:
                        x = sol.y[:,-1]
                        # print("Getting x here: ", x)
                    
                    self.cable_is_slack = self.isslack_multi(x[0:3].reshape((3,1))+ utilslib.QuatToRot(x[6:10]) @ self.pl_params.rho_vec_list, x[13*np.arange(1, self.nquad+1)+np.array([[0],[1],[2]])],self.pl_params.cable_length)

  def cooperative_check_inelastic(self, x):
      # DESCRIPTION:
      # cooperative_check_inelastic check if any cables are slack or taut
      
      # INPUTS:
      # x     - ndarray, state of the multi-robot-payload system
      
      # OUTPUTS:
      # collision_condition   - self.nquad x 1 ndarray, True False value as collision condition
      # state assignment
      nquad = self.nquad
      rho_vec_list = self.pl_params.rho_vec_list
      pl_pos = x[0:3].reshape((3, 1))
      pl_vel = x[3:6].reshape((3, 1))
      pl_rot = utilslib.QuatToRot(x[6:10])
      # pl_rot = pl_rot.T
      attach_pos = pl_pos + pl_rot @ rho_vec_list
      attach_vel = pl_vel + pl_rot @ utilslib.vec2asym(x[10:13]) @ rho_vec_list
      robot_pos = x[13*np.arange(1, nquad + 1) + np.array([[0], [1], [2]])]
      robot_vel = x[13*np.arange(1, nquad + 1) + np.array([[3], [4], [5]])]
      xi = (attach_pos - robot_pos) / np.linalg.norm(attach_pos - robot_pos, 2, 0)
      
      # check collision condition
      # print("xi is")
      # print(xi)
      # print("relative vel")
      # print((attach_vel - robot_vel))
      # print("product is ")
      # print(xi * (attach_vel - robot_vel))
      positive_attach_robot_vel = (np.sum(xi * (attach_vel - robot_vel), 0) > 1e-3)
      taut_condition = self.istaut(attach_pos, robot_pos, self.pl_params.cable_length)
      collision_condition = np.empty((taut_condition.shape))
      for i in range(taut_condition.shape[0]):
        collision_condition[i] = positive_attach_robot_vel[i] and taut_condition[i]
      return collision_condition, xi                    