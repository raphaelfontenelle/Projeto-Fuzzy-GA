from cv_bridge import CvBridge
from std_msgs.msg import Float64MultiArray
import rclpy
import math
from warnings import catch_warnings
import cv2
from geometry_msgs.msg import Twist
from tf2_ros.buffer import Buffer
import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from turtlesim.srv import Spawn
from geometry_msgs.msg import Pose2D, TwistStamped
from tf2_geometry_msgs import PointStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float64MultiArray
import image_geometry
import tf2_ros
import tf2_geometry_msgs
from sensor_msgs.msg import CameraInfo
from tf2_geometry_msgs import Vector3Stamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import random
import numpy
from deap import algorithms, base, creator, tools
import math
qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)

class MyNode(Node):

    def __init__(self):
        super().__init__('turtle_tf2_frame_listener')
        self.model = image_geometry.PinholeCameraModel()
        self.pub2 = self.create_publisher(Float64MultiArray, '/servo_controller/commands', 10)
        self.subscription = self.create_subscription(
            Image,
            '/camera_name2/image_raw',
            self.image_callback,
            qos_policy)
            
        self.bridge = CvBridge()
        
        self.subscription = self.create_subscription(
            CameraInfo,
            '/camera_name2/camera_info',
            self.camera_info_callback,
            qos_policy)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        print('Running')
        self.pos = 0
        
        self.pub = self.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', 10)
        
        self.pubGoal = self.create_publisher(Pose2D, 'goal', 10)
        self.pubAng = self.create_publisher(Float64MultiArray, '/servo_controller/commands', 10)
        #self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.broadcaster = TransformBroadcaster(self)
 
    def camera_info_callback(self, msg):
        print("Got camera info")
        self.model.fromCameraInfo(msg)
        
        
    def image_callback(self, msg):
        print("Cheogou aqui")
        cv_image = cv2.cvtColor(self.bridge.imgmsg_to_cv2(msg), cv2.COLOR_BGR2RGB)
        
        img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        #img2 = cv2.inRange(img, ( 0, 200, 50),	(40, 255, 255))
        
        img2 = cv2.inRange(img, ( 0,150,0),
		(108,255,255))
		
        cv2.imshow('segmented',img2)
        cv2.imshow('hsv',img)
        cv2.imshow('rgb',cv_image)
        cv2.waitKey(1)

        m = cv2.moments(img2)
        if m['m00'] == 0: 
            print("no ball")
            return
        cx = m['m10']/m['m00']
        cy = m['m01']/m['m00']
        
        ma  = Float64MultiArray()
        h = img2.shape[1]/2
        de = h-cx
        self.pos += 0.0001*de
        ma.data = [self.pos]
        self.pubAng.publish(ma)
        
        #print("angle: ", self.pos)
        
        left, top, width, height = cv2.boundingRect(img2)
        base = top + height
        print("Chegou ponto 1")
        print("cx: "+ str(cx) )
        print("base: " + str(base))
        print("model:" + str(self.model))
        try:
            line = self.model.projectPixelTo3dRay((cx,base))
        except Exception as e:
            print("Ocorru um erro: " + str(e))
            return
        print("Chegou ponto 2")
        ps = PointStamped() 
        ps.header.frame_id = 'camera_link'
        ps.point.x = 0.0
        ps.point.y = 0.0
        ps.point.z = 0.0
 
        l2 = Vector3Stamped()
        l2.header.frame_id = 'camera_link'
        l2.vector.x = line[0]
        l2.vector.y = line[1]
        l2.vector.z = line[2]
        angulo = 0
        try:
            vect2 = self.tf_buffer.transform(l2, "base_link")
            print("vect2.vector.y" + str(vect2.vector.y) + "vect2.vector.x :" + str(vect2.vector.x))
            angulo = math.atan2(vect2.vector.y, vect2.vector.x)
            print("angulo:" + str(angulo))
            l2 = self.tf_buffer.transform(l2, "odom")
            ps = self.tf_buffer.transform(ps, "odom")
        except Exception as e:
            print("Ocorru um erro: " + str(e))
            #return
        k = -ps.point.z / l2.vector.z
        vx = ps.point.x + (l2.vector.x)*k
        vy = ps.point.y + (l2.vector.y)*k
      
        #print("goal: ", goal)
        print("vc:" + str(vx))
        
        angulo_entrada = ctrl.Antecedent(np.arange(-3.14, 3.14, 0.01), 'angulo_entrada')

        #angulo_entrada.universe

        angulo_saida = ctrl.Consequent(np.arange(-3.14, 3.14, 0.01), 'angulo_saida')

        angulo_entrada.automf(number = 5, names = ['muitoEsquerda', 'Esquerda', 'Centro', 'Direita', 'muitoDireita'])

        angulo_saida.automf(number = 5, names=['muitoEsquerda', 'Esquerda', 'Centro', 'Direita', 'muitoDireita'])

        angulo_entrada['muitoEsquerda'] = fuzz.trimf(angulo_entrada.universe, [-3.13, -2.5, -2])
        angulo_entrada['Esquerda'] = fuzz.trimf(angulo_entrada.universe, [-2.5, -1.5, -0.1])
        angulo_entrada['Centro'] = fuzz.trimf(angulo_entrada.universe, [-0.5, 0.0, 0.5])
        angulo_entrada['Direita'] = fuzz.trimf(angulo_entrada.universe, [0.0, 1.0, 2])
        angulo_entrada['muitoDireita'] = fuzz.trimf(angulo_entrada.universe, [1.5, 1.8, 3.1])

        angulo_saida['muitoEsquerda'] = fuzz.trimf(angulo_saida.universe, [-2.5, -2, -1.6])
        angulo_saida['Esquerda'] = fuzz.trimf(angulo_saida.universe, [-1.8, -1.0, -0.1])
        angulo_saida['Centro'] = fuzz.trimf(angulo_saida.universe, [-0.2, 0.0, 0.2])
        angulo_saida['Direita'] = fuzz.trimf(angulo_saida.universe, [0.1, 1.0, 2.5])
        angulo_saida['muitoDireita'] = fuzz.trimf(angulo_saida.universe, [1.6, 2, 2.5])

        #angulo_saida.view()

        regra1 = ctrl.Rule(angulo_entrada['muitoEsquerda'], angulo_saida['muitoEsquerda'])
        regra2 = ctrl.Rule(angulo_entrada['Esquerda'], angulo_saida['Esquerda'])
        regra3 = ctrl.Rule(angulo_entrada['Centro'], angulo_saida['Centro'])
        regra4 = ctrl.Rule(angulo_entrada['Direita'], angulo_saida['Direita'])
        regra5 = ctrl.Rule(angulo_entrada['muitoDireita'], angulo_saida['muitoDireita'])

        sistema_controle = ctrl.ControlSystem([regra1, regra2, regra3, regra4, regra5])

        sistema = ctrl.ControlSystemSimulation(sistema_controle)

        sistema.input['angulo_entrada'] = angulo

        #print(sistema.output['angulo_saida'])
        #print("angulo: " + str(angulo))
        sistema.compute()
        #saida.view(sim=sistema)
     
     
        try:
    
            msg = TwistStamped()
            velocidade = sistema.output['angulo_saida']
            print("angulo_saida:" + str(angulo_saida ))
            msg.twist.linear.x = velocidade # velocidade linear
            print("Cegou no ponto de calcular")
            sistema.compute()
            value = float(sistema.output["angulo_saida"])
            print("cal:" +  str(value))
            msg.twist.angular.z = float(value) #velocidade angular
            
        except Exception as e:
            print("Ocorru um erro: " + str(e))
        self.pub.publish(msg)
        global ba1
        global ba2 
        global bt1
        
          # Função Objetivo
        def objective_function(individual):
            print("funcao objetivo")
            ba1 = individual[0]
            ba2 = individual[1]
            bt1 = individual[2]
            try:
                sistema.compute()
                     
            except Exception as e:
                print("Ocorru um erro: " + str(e))
            return angulo
     
        creator.create("FitnessMin", base.Fitness, weights=(-1.0, -1.0 -1.0))       # função objetivo: nome, tipo(f.o.), peso de cada objetivo (no caso só um objetivo)
        creator.create("Individual", list,  fitness=creator.FitnessMin)   # indivíduo
     
        toolbox = base.Toolbox()

        sup = 50
        inf = 0

        toolbox.register("attr_real", random.uniform, inf, sup)
        toolbox.register("individual", tools.initRepeat, creator.Individual, toolbox.attr_real, 3)  
        toolbox.register("population", tools.initRepeat, list, toolbox.individual)   
     
     
        toolbox.register('evaluate', objective_function)
        toolbox.register("mate", tools.cxSimulatedBinaryBounded, eta= 0.5, low= inf, up= sup) 
        toolbox.register("mutate", tools.mutGaussian, mu=0, sigma=1, indpb=0.05)
        toolbox.register('select', tools.selTournament, tournsize=3)
     
        pop = toolbox.population(n=100)                            # inicialização da pop
        hof = tools.HallOfFame(1)                                # melhor indivíduo
        stats = tools.Statistics(lambda ind: ind.fitness.values)  # estatísticas
        stats.register("avg", numpy.mean)
        stats.register("std", numpy.std)
        stats.register("min", numpy.min)
        stats.register("max", numpy.max)
     
     
        pop, log = algorithms.eaSimple(pop, toolbox, cxpb=0.5, mutpb=0.1, ngen=10, stats=stats, halloffame=hof, verbose=True)
     
        print("Melhor Indivíduo:")
        print(hof[0])

     
        angulo_entrada_v = ctrl.Antecedent(np.arange(-3.14, 3.14, 0.01), 'angulo_entrada_v')

        velocidade_saida = ctrl.Consequent(np.arange(0.0, 10.0, 0.0001), 'velocidade_saida')

        angulo_entrada_v.automf(number = 5, names = ['muitoEsquerda', 'Esquerda', 'Centro', 'Direita', 'muitoDireita'])

        velocidade_saida.automf(number = 4, names=['muitoRapido', 'Rapido','Centro', 'Devagar', 'MuitoDevagar'])

        angulo_entrada_v['muitoEsquerda'] = fuzz.trimf(angulo_entrada_v.universe, [-3.13, -2.5, -2])
        angulo_entrada_v['Esquerda'] = fuzz.trimf(angulo_entrada_v.universe, [-2.5, -1.5, -0.1])
        angulo_entrada_v['Centro'] = fuzz.trimf(angulo_entrada_v.universe, [-0.5, 0.3, 0.5])
        angulo_entrada_v['Direita'] = fuzz.trimf(angulo_entrada_v.universe, [0.0, 1.0, 2])
        angulo_entrada_v['muitoDireita'] = fuzz.trimf(angulo_entrada_v.universe, [1.5, 1.8, 3.1])

        velocidade_saida['muitoRapido'] = fuzz.trimf(velocidade_saida.universe, [6, 8, 10])
        velocidade_saida['Rapido'] = fuzz.trimf(velocidade_saida.universe, [4, 5, 7])
        velocidade_saida['Centro'] = fuzz.trimf(velocidade_saida.universe, [3, 4, 6])
        velocidade_saida['Devagar'] = fuzz.trimf(velocidade_saida.universe, [2, 3, 4])
        velocidade_saida['MuitoDevagar'] = fuzz.trimf(velocidade_saida.universe, [1, 2, 3])


        regra1_v = ctrl.Rule(angulo_entrada_v['muitoEsquerda'], velocidade_saida['Devagar'])
        regra2_v = ctrl.Rule(angulo_entrada_v['Esquerda'], velocidade_saida['Rapido'])
        regra3_v = ctrl.Rule(angulo_entrada_v['Centro'], velocidade_saida['muitoRapido'])
        regra4_v = ctrl.Rule(angulo_entrada_v['Direita'], velocidade_saida['Rapido'])
        regra5_v = ctrl.Rule(angulo_entrada_v['muitoDireita'], velocidade_saida['Devagar'])

        sistema_controle_v = ctrl.ControlSystem([regra1_v, regra2_v, regra3_v, regra4_v, regra5_v])

        sistema_v = ctrl.ControlSystemSimulation(sistema_controle_v)

        sistema_v.input['angulo_entrada_v'] = angulo


        try:
            sistema_v.compute()
            msg = TwistStamped()
            velocidade = sistema_v.output['velocidade_saida']/10
            print("velocidade:" + str(velocidade ))
            msg.twist.linear.x = velocidade # velocidade linear
            print("Cegou no ponto de calcular")
            sistema.compute()
            value = float(sistema.output["angulo_saida"])
            print("cal:" +  str(value))
            msg.twist.angular.z = float(value) #velocidade angular
            
        except Exception as e:
            print("Ocorru um erro: " + str(e))
        self.pub.publish(msg)
        
        
        
def main(args=None):
    rclpy.init(args=args)
    my_node = MyNode()
    rclpy.spin(my_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    my_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
