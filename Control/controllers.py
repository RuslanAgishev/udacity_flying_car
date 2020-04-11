import math

class OpenLoopController:
    
    def __init__(self, vehicle_mass, initial_state, mass_error=1.0): 
        self.vehicle_mass  = vehicle_mass * mass_error
        self.vehicle_state = initial_state
        self.g = 9.81
    
    def thrust_control(self, target_z, dt):
        """
        Returns a thrust which will be commanded to 
        the vehicle. This thrust should cause the vehicle
        to be at target_z in dt seconds.
        
        The controller's internal model of the vehicle_state
        is also updated in this method.
        """
        # 1. find target velocity needed to get to target_z
        current_z, current_z_dot = self.vehicle_state
        delta_z = target_z - current_z
        target_z_dot = delta_z / dt
        
        # 2. find target acceleration needed
        delta_z_dot = target_z_dot - current_z_dot
        target_z_dot_dot = delta_z_dot / dt
        
        # 3. find target NET force
        target_f_net = target_z_dot_dot * self.vehicle_mass
        
        # 4. find target thrust. Recall this equation:
        #    F_net = mg - thrust
        thrust = self.vehicle_mass * self.g - target_f_net
        
        # 5. update controller's internal belief of state
        self.vehicle_state += np.array([delta_z, delta_z_dot])
        
        return thrust 
    
    
    
class PController:
    
    def __init__(self, k_p, m):
        self.k_p = k_p 
        self.vehicle_mass = m   
        self.g = 9.81
    
    def thrust_control(self, z_target, z_actual):
        # TODO - implement this method!
        
        err = z_target - z_actual
        
        # u_bar is what we want vertical acceleration to be
        u_bar = self.k_p * err 
        
        # u is the thrust command which will cause u_bar 
        u = self.vehicle_mass * (self.g - u_bar)
        
        return u
    
    
    
class PDController:
    
    def __init__(self, k_p, k_d, m):
        self.k_p = k_p
        self.k_d = k_d
        self.vehicle_mass = m
        self.g = 9.81
    
    def thrust_control(self,
                z_target, 
                z_actual, 
                z_dot_target, 
                z_dot_actual,
                z_dot_dot_ff=0.0):
        err = z_target - z_actual
        err_dot = z_dot_target - z_dot_actual
        u_bar = self.k_p * err + self.k_d * err_dot + z_dot_dot_ff
        u = self.vehicle_mass * (self.g - u_bar) 
        return u
    
    
class PIDController:
    
    def __init__(self, k_p, k_d, k_i, m):
        self.k_p = k_p
        self.k_d = k_d
        self.k_i = k_i
        self.vehicle_mass = m
        self.g = 9.81
        self.integrated_error = 0.0
        
    def thrust_control(self,
                z_target, 
                z_actual, 
                z_dot_target, 
                z_dot_actual,
                dt,
                z_dot_dot_ff=0.0):
        
        err = z_target - z_actual
        err_dot = z_dot_target - z_dot_actual
        self.integrated_error += err * dt
        
        p = self.k_p * err
        i = self.integrated_error * self.k_i
        d = self.k_d * err_dot
         
        u_bar = p + i + d + z_dot_dot_ff
        u = self.vehicle_mass * (self.g - u_bar)
        return u
        
        
class LinearCascadingController:
    
    def __init__(self,
                 m,   # needed to convert u1_bar to u1
                 I_x, # needed to convert u2_bar to u2
                 z_k_p=1.0,   
                 z_k_d=1.0,   
                 y_k_p=1.0,
                 y_k_d=1.0,
                 phi_k_p=1.0,
                 phi_k_d=1.0):
        
        self.z_k_p = z_k_p
        self.z_k_d = z_k_d   
        self.y_k_p = y_k_p
        self.y_k_d = y_k_d
        self.phi_k_p = phi_k_p
        self.phi_k_d = phi_k_d
        
        self.g = 9.81
        self.I_x = I_x
        self.m = m

    def altitude_controller(self, 
                    z_target, 
                    z_actual, 
                    z_dot_target, 
                    z_dot_actual,
                    z_dot_dot_target,
                    phi_actual, # unused parameter. Ignore for now.
                    ):
        """
        A PD controller which commands a thrust (u_1) 
        for the vehicle. 
        """
        z_err = z_target - z_actual
        z_err_dot = z_dot_target - z_dot_actual

        p_term = self.z_k_p * z_err
        d_term = self.z_k_d * z_err_dot

        u_1_bar = p_term + d_term + z_dot_dot_target
        u_1 = self.m* (self.g - u_1_bar)

        return u_1

    
    def lateral_controller(self, 
                        y_target, 
                        y_actual, 
                        y_dot_target, 
                        y_dot_actual,
                        u_1=None, # unused parameter. Ignore for now.
                        y_dot_dot_ff=0.0,
                        ):
        """
        A PD controller which commands a target roll 
        angle (phi_commanded).
        """
        y_err = y_target - y_actual
        y_err_dot = y_dot_target - y_dot_actual

        p_term = self.y_k_p * y_err
        d_term = self.y_k_d * y_err_dot
        
        y_dot_dot_target = p_term + d_term + y_dot_dot_ff
        phi_commanded = y_dot_dot_target / self.g

        return phi_commanded 



    def attitude_controller(self, 
                            phi_target, 
                            phi_actual, 
                            phi_dot_actual,
                            phi_dot_target=0.0
                           ):
        """
        A PD controller which commands a moment (u_2)
        about the x axis for the vehicle.
        """

        phi_err = phi_target - phi_actual
        phi_err_dot = phi_dot_target - phi_dot_actual

        p_term = self.phi_k_p * phi_err
        d_term = self.phi_k_d * phi_err_dot
        
        u_2_bar = p_term + d_term
        u_2 = (u_2_bar) * self.I_x

        return u_2
    
class NonLinearCascadingController:
    
    def __init__(self,
                 m,
                 I_x,
                 z_k_p=1.0,   
                 z_k_d=1.0,   
                 y_k_p=1.0,
                 y_k_d=1.0,
                 phi_k_p=1.0,
                 phi_k_d=1.0):
        
        self.z_k_p = z_k_p
        self.z_k_d = z_k_d   
        self.y_k_p = y_k_p
        self.y_k_d = y_k_d
        self.phi_k_p = phi_k_p
        self.phi_k_d = phi_k_d
        
        self.g = 9.81
        self.I_x = I_x
        self.m = m
                    
    def altitude_controller(self, 
                    z_target, 
                    z_actual, 
                    z_dot_target, 
                    z_dot_actual,
                    z_dot_dot_target,
                    phi_actual):

        z_err = z_target - z_actual
        z_err_dot = z_dot_target - z_dot_actual

        p_term = self.z_k_p * z_err
        d_term = self.z_k_d * z_err_dot

        u_1_bar = p_term + d_term + z_dot_dot_target

        u_1 = self.m* (self.g - u_1_bar)/ math.cos(phi_actual)

        return u_1
    
    
    def lateral_controller(self, 
                        y_target,
                        y_actual, 
                        y_dot_target, 
                        y_dot_actual,
                        u_1,
                        y_dot_dot_ff=0.0,
                        ):
        y_err = y_target - y_actual
        y_err_dot = y_dot_target - y_dot_actual

        p_term = self.y_k_p * y_err
        d_term = self.y_k_d * y_err_dot
        y_dot_dot_target = p_term + d_term + y_dot_dot_ff
        
        x = self.m * y_dot_dot_target / u_1
        x = min(0.99,max(x,-0.99))
        phi_commanded = math.asin( x )

        return phi_commanded 
    
    
    def attitude_controller(self, 
                        phi_target, 
                        phi_actual, 
                        phi_dot_actual,
                        phi_dot_target=0.0 
                        ):
        phi_err = phi_target - phi_actual
        phi_err_dot = phi_dot_target - phi_dot_actual

        p_term = self.phi_k_p * phi_err
        d_term = self.phi_k_d * phi_err_dot

        u_2 = (p_term + d_term ) * self.I_x

        return u_2
        