import rclpy

import numpy as np 
from sensor_msgs.msg import JointState


class nodeNumericMethod():
    def __init__(self):
        
        self.L1 = 0.018  
        self.L2 = -0.047 
        self.L3 = -0.025
        self.L4 = -0.033
        self.L5 = -0.090
        

    def fkine(self, theta1, theta2, theta3, theta4, theta5):
        
        #print(f"Angulos recibidos: \n Q1 = {np.degrees(theta1)}, \n Q2 = {np.degrees(theta2)},\n Q3 = {np.degrees(theta3)},\n Q4 = {np.degrees(theta4)},\n Q5 = {np.degrees(theta5)}")

        self.L1 = 0.018  
        self.L2 = -0.047 
        self.L3 = -0.025
        self.L4 = -0.033
        self.L5 = -0.090

        self.d1 = self.L1
        self.d2 = 0
        self.d3 = 0
        self.d4 = 0
        self.d5 = 0

        self.a1 = 0
        self.a2 = self.L2    
        self.a3 = self.L3
        self.a4 = self.L4
        self.a5 = self.L5    

        self.alpha1 = np.radians(-90)
        self.alpha2 = np.radians(0)
        self.alpha3 = np.radians(0)
        self.alpha4 = np.radians(0)
        self.alpha5 = np.radians(0)

        self.T_new_y = np.array([[np.cos(np.radians(90)), 0, np.sin(np.radians(90)), 0],
                                 [0, 1, 0, 0],
                                 [-np.sin(np.radians(90)), 0, np.cos(np.radians(90)), 0],
                                 [0, 0, 0, 1]])

        self.T1 = self.denavit(theta1, self.d1, self.a1, self.alpha1)
        self.T2 = self.denavit(theta2, self.d2, self.a2, self.alpha2)
        self.T3 = self.denavit(theta3, self.d3, self.a3, self.alpha3)
        self.T4 = self.denavit(theta4, self.d4, self.a4, self.alpha4)
        self.T5 = self.denavit(theta5, self.d5, self.a5, self.alpha5)

        self.T_total = np.dot(np.dot(np.dot(np.dot(self.T1, self.T2), self.T3), self.T4), self.T5)
        self.T_total = np.dot(self.T_new_y, self.T_total)

        PX = self.T_total[0,3] 
        PY = self.T_total[1,3]
        PZ = self.T_total[2,3]

        P_array = np.array([PX, PY, PZ, 0])

        #print(f"Valores en posiciones: \n X = {PX}, \n Y = {PY}, \n Z = {PZ}")

        return P_array


    def denavit(self, teta, d, a, alfa):
        dh = np.array([[np.cos(teta), -np.cos(alfa)*np.sin(teta), np.sin(alfa)*np.sin(teta), a*np.cos(teta)],
                      [np.sin(teta), np.cos(alfa)*np.cos(teta), -np.sin(alfa)*np.cos(teta), a*np.sin(teta)],
                      [0, np.sin(alfa), np.cos(alfa), d],
                      [0, 0, 0, 1]])
        
        return dh


    def numeric_method(self, desired_point):
        
        #print("ENTRAR AL METODO NUMERICO")
        theta1 = np.radians(0)   
        theta2 = np.radians(180)
        theta3 = np.radians(45)
        theta4 = np.radians(-90)
        theta5 = np.radians(90)

        #desired_point = np.array([0.051, 0.0, -0.162, 0.0])
        
        delta = 0.0001
        epsilon = 1e-3
        max_iter = 100

        for i in range(max_iter):
            q = np.array([theta1, theta2, theta3, theta4, theta5]) 


            JT = 1/delta*np.array([
                self.fkine(theta1+delta, theta2, theta3, theta4, theta5) - self.fkine(theta1, theta2, theta3, theta4, theta5),
                self.fkine(theta1, theta2+delta, theta3, theta4, theta5) - self.fkine(theta1, theta2, theta3, theta4, theta5),
                self.fkine(theta1, theta2, theta3+delta, theta4, theta5) - self.fkine(theta1, theta2, theta3, theta4, theta5),
                self.fkine(theta1, theta2, theta3, theta4+delta, theta5) - self.fkine(theta1, theta2, theta3, theta4, theta5),
                self.fkine(theta1, theta2, theta3, theta4, theta5+delta) - self.fkine(theta1, theta2, theta3, theta4, theta5)
            ])

            #print(f"EL JACOBIANO  \n {JT}")

            J = JT.transpose()

            f = self.fkine(theta1, theta2, theta3, theta4, theta5)
            e = desired_point - f
            q = q + np.dot(np.linalg.pinv(J), e)
            #print(f"EL ERROR \n {e}")
            #print("\n")
            #print(f"EL JACOBIANO PSEUDO INVERSA \n {np.linalg.pinv(J)}")

            q[1] = np.radians(180)
            q[3] = np.radians(-90)
        

            #print(f"q: {q}, Error: {e}")        


            theta1 = q[0]
            theta2 = q[1]
            theta3 = q[2] #- np.radians(90)
            theta4 = q[3] 
            theta5 = q[4] #- np.radians(90)

            # Condicion de termino
            if (np.linalg.norm(e) < epsilon):
                break

            """               
            if theta1 <= np.radians(-90):
                theta1 = np.radians(10)

            if theta1 >= np.radians(10):
                theta1 = np.radians(-90)

            if theta3 <= np.radians(-45):
                theta3 = np.radians(90)

            if theta3 >= np.radians(90):
                theta3 = np.radians(-45)

            if theta5 <= np.radians(-90):
                theta5 = np.radians(45)            

            if theta5 >= np.radians(45):
                theta5 = np.radians(-90) 
            """
            if theta1 <= np.radians(-180):
                theta1 = np.radians(180)

            if theta1 >= np.radians(180):
                theta1 = np.radians(-180)

            if theta3 <= np.radians(-180):
                theta3 = np.radians(180)

            if theta3 >= np.radians(180):
                theta3 = np.radians(-180)

            if theta5 <= np.radians(-180):
                theta5 = np.radians(180)            

            if theta5 >= np.radians(180):
                theta5 = np.radians(-180)

            #q = np.clip(q, np.radians(q_min), np.radians(q_max))

        #self.get_logger().info(f"Valores finales de NEWTON: \n Q1 = {np.degrees(theta1)}, \n Q2 = {np.degrees(theta2)},\n Q3 = {np.degrees(theta3)},\n Q4 = {np.degrees(theta4)},\n Q5 = {np.degrees(theta5)},\n f={f},\n max iter={i}")
        #print(f"Valores finales de NEWTON: \n Q1 = {np.degrees(theta1)}, \n Q2 = {np.degrees(theta2)},\n Q3 = {np.degrees(theta3)},\n Q4 = {np.degrees(theta4)},\n Q5 = {np.degrees(theta5)},\n max iter={i}")

        if i <= 99:

            delta = 0.0001
            epsilon = 1e-3
            max_iter = 10000
            alpha = 10
            i = 0

            for i in range(max_iter):
                q = np.array([theta1, theta2, theta3, theta4, theta5]) 

                JT = 1/delta*np.array([
                    self.fkine(theta1+delta, theta2, theta3, theta4, theta5) - self.fkine(theta1, theta2, theta3, theta4, theta5),
                    self.fkine(theta1, theta2+delta, theta3, theta4, theta5) - self.fkine(theta1, theta2, theta3, theta4, theta5),
                    self.fkine(theta1, theta2, theta3+delta, theta4, theta5) - self.fkine(theta1, theta2, theta3, theta4, theta5),
                    self.fkine(theta1, theta2, theta3, theta4+delta, theta5) - self.fkine(theta1, theta2, theta3, theta4, theta5),
                    self.fkine(theta1, theta2, theta3, theta4, theta5+delta) - self.fkine(theta1, theta2, theta3, theta4, theta5)
                ])

                J = JT.transpose()
                f = self.fkine(theta1, theta2, theta3, theta4, theta5)
                e = desired_point - f
                q = q + alpha*np.dot(JT, e)
                #q = q + alpha*np.dot(np.linalg.pinv(J), e)

                #print(f"q: {q}, Error: {e}")        

                q[1] = np.radians(180)
                q[3] = np.radians(-90)

                theta1 = q[0]
                theta2 = q[1]
                theta3 = q[2] #- np.radians(90)
                theta4 = q[3] 
                theta5 = q[4] #- np.radians(90)

                print(f"MAXIMA CANTIDAD DE ITERACIONES= {i}")
                
                # Condicion de termino
                if (np.linalg.norm(e) < epsilon):
                    break

                """        
                if theta1 <= np.radians(-90):
                    theta1 = np.radians(10)

                if theta1 >= np.radians(10):
                    theta1 = np.radians(-90)

                if theta3 <= np.radians(-45):
                    theta3 = np.radians(90)

                if theta3 >= np.radians(90):
                    theta3 = np.radians(-45)

                if theta5 <= np.radians(-90):
                    theta5 = np.radians(45)            

                if theta5 >= np.radians(45):
                    theta5 = np.radians(-90) 
                """
                if theta1 <= np.radians(-180):
                    theta1 = np.radians(180)

                if theta1 >= np.radians(180):
                    theta1 = np.radians(-180)

                if theta3 <= np.radians(-180):
                    theta3 = np.radians(180)

                if theta3 >= np.radians(180):
                    theta3 = np.radians(-180)

                if theta5 <= np.radians(-180):
                    theta5 = np.radians(180)            

                if theta5 >= np.radians(180):
                    theta5 = np.radians(-180)

                #q = np.clip(q, np.radians(q_min), np.radians(q_max))
        #self.get_logger().info(f"Valores finales de GRADIENTE: \n Q1 = {np.degrees(theta1)}, \n Q2 = {np.degrees(theta2)},\n Q3 = {np.degrees(theta3)},\n Q4 = {np.degrees(theta4)},\n Q5 = {np.degrees(theta5)},\n f={f},\n max iter={i}")
        #print(f"Valores finales de GRADIENTE: \n Q1 = {np.degrees(theta1)}, \n Q2 = {np.degrees(theta2)},\n Q3 = {np.degrees(theta3)},\n Q4 = {np.degrees(theta4)},\n Q5 = {np.degrees(theta5)},\n max iter={i}")
        q = np.array([theta1, theta3, theta5-np.radians(90), 0.0])
        qfinal = np.rad2deg(q)
        return q

        #msg = JointState()
        #msg.name = ["hombro_DA_joint", "codo_DA_joint", "muneca_DA_joint", "end_effector_joint"]
        #msg.position = [theta1, theta3, theta5-np.radians(90), 0.0]
        #msg.header.stamp = self.get_clock().now().to_msg()
        #self.rviz_publisher.publish(msg)

def main():
    #rclpy.init(args=args)
    node = nodeNumericMethod()
    desired = np.array([0.051, 0.0, -0.162, 0.0])
    angles = node.numeric_method(desired)
    #print("Ángulos resultantes:", angles)
    #rclpy.spin(node)
    #node.destroy_node()
    #rclpy.shutdown()

if __name__ == '__main__':
    main()