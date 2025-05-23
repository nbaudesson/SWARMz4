import time
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64
from cannon_interfaces.action import Cannon
from .gz_tracker import GazeboPosesTracker
# from game_master.gazebo_subscriber import GazeboPosesTracker
from action_msgs.msg import GoalStatus

class CannonServer(Node):

    def __init__(self):
        super().__init__('cannon_server')

        self.rate = self.create_rate(100, self.get_clock())
        # Déclaration des paramètres
        self.declare_parameter("speed", 1.0)
        self.declare_parameter("tolerance", 0.017)
        self.speed = self.get_parameter("speed").value
        self.tolerance = self.get_parameter("tolerance").value
        
        # Fix namespace handling to avoid double slashes
        self.namespace = self.get_namespace()
        if self.namespace.endswith('/'):
            self.namespace = self.namespace[:-1]  # Remove trailing slash
            
        # Now create publishers with properly formatted topic names
        self.pitch_pub = self.create_publisher(Float64, f'{self.namespace}/cannon_pitch_cmd', 10)
        self.yaw_pub = self.create_publisher(Float64, f'{self.namespace}/cannon_yaw_cmd', 10)

        # Initialisation des poses
        self.pitch = 0.0
        self.yaw = 0.0
    
        # Initialisation de targets
        self.target_pitch = 0.0
        self.target_yaw = 0.0

        # Goal en cours
        self._current_goal_handle = None
        self._current_goal_request = None

        # Interface Gazebo
        self.gz_tracker = GazeboPosesTracker(self.namespace, logger=self.get_logger())
        self.update_positions() # Premier appel pour obtenir orientation du cannon 

        # Timer pour mise à jour continue des poses
        self.timer = self.create_timer(0.1, self.update_positions)  # 10Hz

        # Action server
        self._action_server = ActionServer(
            self,
            Cannon,
            'cannon',
            execute_callback=self.execute_callback,
            callback_group=MutuallyExclusiveCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.get_logger().info("Cannon action server initialisé.")

    def update_positions(self):
        try:
            pitch, yaw= self.gz_tracker.get_cannon_rpy()
            # # Mettre à jour les valeurs de pitch et yaw
            self.pitch = pitch
            self.yaw = yaw
        except Exception as e:
            self.get_logger().error(f"Erreur de lecture de pose Gazebo : {e}")

    def goal_callback(self, goal_request):

        self.target_pitch = goal_request.pitch
        self.target_yaw = goal_request.yaw

        # Définition des bornes autorisées
        min_pitch, max_pitch = -1.57, 1.57
        min_yaw, max_yaw = -3.14, 3.14
        
        if not (min_pitch <= self.target_pitch <= max_pitch):
            self.get_logger().warn(
                f" Pitch {self.target_pitch:.2f} rad hors bornes autorisées [{min_pitch}, {max_pitch}]. Requête refusée."
            )
            return GoalResponse.REJECT

        if not (min_yaw <= self.target_yaw <= max_yaw):
            self.get_logger().warn(
                f" Yaw {self.target_yaw:.2f} rad hors bornes autorisées [{min_yaw}, {max_yaw}]. Requête refusée."
            )
            return GoalResponse.REJECT

        # Comparer avec goal actuel
        if self._current_goal_handle is not None and self._current_goal_handle.status == GoalStatus.STATUS_EXECUTING:
            current = self._current_goal_request
            if (current.pitch == goal_request.pitch and
                current.yaw == goal_request.yaw):
                self.get_logger().info("Goal identique déjà en cours. Rejeté.")
                return GoalResponse.REJECT
            else:
                self.get_logger().warn("Nouveau goal reçu. Abandon de l'ancien.")
                self._current_goal_handle.abort()

        self.get_logger().info(f"Nouvelle commande reçue : pitch= {self.target_pitch:.2f} rad, yaw= {self.target_yaw:.2f} rad")
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        self.get_logger().warn("Demande d'annulation reçue.")
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info("Exécution de la commande en cours...")

        self._current_goal_handle = goal_handle
        self._current_goal_request = goal_handle.request


        # Débug : Log des valeurs au début de l'exécution
        # self.get_logger().info(f"Target: pitch={self.target_pitch:.3f}, yaw={self.target_yaw:.3f}")
        # self.get_logger().info(f"Current: pitch={self.pitch:.3f}, yaw={self.yaw:.3f}")

        self.get_logger().info(f"Target: pitch= {self.target_pitch:.3f} rad, yaw= {self.target_yaw:.3f} rad, Current: pitch= {self.pitch:.3f} rad, yaw= {self.yaw:.3f} rad")

        feedback = Cannon.Feedback()
        result = Cannon.Result()

        # Le timeout sera la valeur maximale entre les deux
        timeout = 25

        self.get_logger().info(f"Temps de durée max pour l'exécution de l'action : {timeout:.3f} secondes")

        # Calculer le temps de fin du timeout
        end_time = time.time() + timeout  # Calculer le temps à partir duquel le timeout se déclenche

        # Début de l'execution de l'action server 
        start = time.time() 

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.stop_all()
                goal_handle.canceled()
                self.get_logger().warn("Commande annulée.")
                return result

            # Si goal abandonné, arrêt de l'exécution et on retourne le résultat
            if goal_handle.status != GoalStatus.STATUS_EXECUTING:
                # self.get_logger().warn("Goal actuel abandonné !") # Vérification que l'action a bien été interrompue 
                return result

            # Vérifier si le timeout est atteint
            if time.time() > end_time:
                self.stop_all()
                goal_handle.abort()  # Marquer l'action comme échouée
                self.get_logger().warn(f"Timeout atteint après {timeout} secondes.")
                return result  # Retourne un résultat d'échec

            error_pitch = self.target_pitch - self.pitch
            error_yaw = self.target_yaw - self.yaw
            
            # Fin si erreurs < tolérance
            if abs(error_pitch) < self.tolerance and abs(error_yaw) < self.tolerance:
                self.stop_all()
                goal_handle.succeed()
                result.success = True
                self.get_logger().info("Objectif atteint.")
                end = time.time() - start 
                self.get_logger().info(f"Action executée en {end} secondes.")
                return result

            # Génération des commandes
            pitch_cmd = self.compute_command(error_pitch)
            yaw_cmd = self.compute_command(error_yaw, is_yaw=True)
            # print("error pitch & yaw : ", error_pitch, error_yaw, "cmd :", pitch_cmd, yaw_cmd)

            # Publication
            self.publish_command(pitch_cmd, yaw_cmd)

            # Feedback
            feedback.current_pitch = self.pitch
            feedback.current_yaw = self.yaw
            goal_handle.publish_feedback(feedback)

            self.rate.sleep()
        
        self.stop_all()
        return result

    def compute_command(self, error, is_yaw=False):
        """
        Calcule la commande proportionnelle avec limitation de vitesse.
        Si `is_yaw` est True, inverse la commande en fonction du signe de l'erreur.
        """
        kp = self.compute_gain(error)
        cmd = kp * error
        # Saturation ou forçage à ±self.speed selon le gain ou la valeur
        if kp == 1.0 or abs(cmd) > self.speed:
            cmd = self.speed if cmd > 0 else -self.speed
        # Inversion de la commande pour `yaw` : si erreur positive, commande négative, et inversement.
        if is_yaw:
            if error > 3.14 or error < -3.14 : 
                cmd = -cmd
        
        return cmd

    def compute_gain(self, error):
        """
        Ajustement dynamique du gain proportionnel.
        """
        abs_error = abs(error)
        if abs_error < self.tolerance:
            return 0.0
        # L'ajustement du KP ici n'est plus forcément nécessaire car la vitesse est relativement lente, donc pas de problème de dépassement 
        # elif abs_error < self.tolerance + 0.01:
        #     return 0.8 #0.4
        # elif abs_error < self.tolerance + 0.05:
        #     return 0.9 #0.6
        return 1.0

    def publish_command(self, pitch_cmd, yaw_cmd):
        pitch_msg = Float64()
        yaw_msg = Float64()
        pitch_msg.data = pitch_cmd
        yaw_msg.data = yaw_cmd
        #print('command pitch :', pitch_msg, 'command yaw :', yaw_cmd)
        self.pitch_pub.publish(pitch_msg)
        self.yaw_pub.publish(yaw_msg)

    def stop_all(self):
        """
        Stoppe tous les mouvements du canon.
        """
        zero_cmd = Float64()
        zero_cmd.data = 0.0
        self.pitch_pub.publish(zero_cmd)
        self.yaw_pub.publish(zero_cmd)


def main(args=None):
    rclpy.init(args=args)
    cannon_action_server = CannonServer()
    executor = MultiThreadedExecutor()
    rclpy.spin(cannon_action_server, executor=executor)
    # rclpy.spin(cannon_action_server)  # sans executor et multithread
    cannon_action_server.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

