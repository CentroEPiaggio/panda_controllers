# #!/usr/bin/env python3
# import rospy
# from gazebo_msgs.msg import ModelState
# from pynput import keyboard # Installa con: pip install pynput

# #./teleop_palla.py
# class PallaTeleop:
#     def __init__(self):
#         rospy.init_node('palla_teleop')
#         self.pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
        
#         # Stato iniziale della palla (coerente con i tuoi parametri C++)
#         self.state = ModelState()
#         self.state.model_name = 'palla'
#         self.state.pose.position.x = 0.3
#         self.state.pose.position.y = -0.35
#         self.state.pose.position.z = 0.53
#         self.state.reference_frame = 'world'
        
#         self.step = 0.001 # Spostamento di 5mm a ogni pressione
#         print("Usa le FRECCE per muovere la palla (X/Y). PagUp/PagDown per Z. ESC per uscire.")

#     def on_press(self, key):
#         try:
#             if key == keyboard.Key.up:
#                 self.state.pose.position.x -= self.step
#             elif key == keyboard.Key.down:
#                 self.state.pose.position.x += self.step
#             elif key == keyboard.Key.left:
#                 self.state.pose.position.y -= self.step
#             elif key == keyboard.Key.right:
#                 self.state.pose.position.y += self.step
#             elif key == keyboard.Key.page_up:
#                 self.state.pose.position.z += self.step
#             elif key == keyboard.Key.page_down:
#                 self.state.pose.position.z -= self.step
#             elif key == keyboard.Key.esc:
#                 return False # Esce dal listener
            
#             self.pub.publish(self.state)
#         except Exception as e:
#             print(e)

#     def run(self):
#         with keyboard.Listener(on_press=self.on_press) as listener:
#             listener.join()

# if __name__ == '__main__':
#     teleop = PallaTeleop()
#     teleop.run()

#!/usr/bin/env python3
import rospy
from gazebo_msgs.msg import ModelState
from pynput import keyboard

class PallaTeleop:
    def __init__(self):
        rospy.init_node('palla_teleop')
        self.pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
        
        self.state = ModelState()
        self.state.model_name = 'palla'
        self.state.pose.position.x = 0.3
        self.state.pose.position.y = -0.35
        self.state.pose.position.z = 0.53
        self.state.reference_frame = 'world'
        
        # Imposta la velocità di movimento in metri al secondo (es. 20 cm/s)
        self.speed = 0.20 
        self.keys_pressed = set()
        
        print("Usa le FRECCE per muovere la palla (X/Y). PagUp/PagDown per Z. ESC per uscire.")
        
        # Loop di controllo a 50Hz per un movimento fluido
        self.timer = rospy.Timer(rospy.Duration(0.02), self.update_and_publish)

    def on_press(self, key):
        self.keys_pressed.add(key)
        if key == keyboard.Key.esc:
            rospy.signal_shutdown("Uscita richiesta dall'utente")
            return False

    def on_release(self, key):
        if key in self.keys_pressed:
            self.keys_pressed.remove(key)

    def update_and_publish(self, event):
        vx, vy, vz = 0.0, 0.0, 0.0

        if keyboard.Key.up in self.keys_pressed:
            vx = -self.speed
        if keyboard.Key.down in self.keys_pressed:
            vx = self.speed
        if keyboard.Key.left in self.keys_pressed:
            vy = -self.speed
        if keyboard.Key.right in self.keys_pressed:
            vy = self.speed
        if keyboard.Key.page_up in self.keys_pressed:
            vz = self.speed
        if keyboard.Key.page_down in self.keys_pressed:
            vz = -self.speed

        # Integrazione della posizione (Spazio = Velocità * Tempo)
        dt = 0.02
        self.state.pose.position.x += vx * dt
        self.state.pose.position.y += vy * dt
        self.state.pose.position.z += vz * dt

        # INFORMAZIONE CRITICA PER L'MPC: Comunichiamo la velocità esatta a Gazebo!
        self.state.twist.linear.x = vx
        self.state.twist.linear.y = vy
        self.state.twist.linear.z = vz

        self.pub.publish(self.state)

    def run(self):
        with keyboard.Listener(on_press=self.on_press, on_release=self.on_release) as listener:
            rospy.spin()

if __name__ == '__main__':
    teleop = PallaTeleop()
    teleop.run()