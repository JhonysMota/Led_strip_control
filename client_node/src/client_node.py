#!/usr/bin/python3

import rospy
from led_strip_driver.srv import LedStripDriverService  # Importa a mensagem do serviço
from std_msgs.msg import String
from std_msgs.msg import Float64

def call_set_outputs_service(output_a, output_b, output_vbat):
    # Espera o serviço estar disponível
    rospy.wait_for_service('/uav/led_strip_driver/set_outputs')
    try:
        # Cria o cliente do serviço
        set_outputs_service = rospy.ServiceProxy('/uav/led_strip_driver/set_outputs', LedStripDriverService)

        # Faz a requisição ao serviço
        response = set_outputs_service(output_a, output_b, output_vbat)

        # Verifica a resposta
        if response.success:
            rospy.loginfo("Outputs configurados com sucesso: output_A: %d, output_B: %d, VBAT: %d",
                          output_a, output_b, output_vbat)
        else:
            rospy.logwarn("Falha ao configurar os outputs: %s", response.message)

    except rospy.ServiceException as e:
        rospy.logerr("Erro ao chamar o serviço: %s", e)

last_conf = 0
pwm_value = 0

def callback_conf (conf):
    print(conf.data)   

    global last_conf
    global pwm_value

    if conf.data > 0.6:
        # Se já está no nível ideal, manter a intensidade atual
        pwm_value = pwm_value  
    else:
        # Se não sabemos ainda se está melhorando, iniciamos a comparação
        if last_conf is not None:
                if conf.data > last_conf:
                    # Se valor de confiança aumentou, aumentamos PWM
                    pwm_value += 10
                else:
                     # Se o valor de confiança diminuiu, reduzimos o PWM
                    pwm_value -= 10
                  
        else:
            # Primeiro ajuste, apenas testamos um aumento inicial
            pwm_value += 10

        # Garante que PWM fique dentro dos limites
        pwm_value = min(max(pwm_value, 10), 100)

        # Envia os valores: output_a = pwm_value, output_b = 0, output_vbat = false
        call_set_outputs_service(pwm_value, 0, False)

        # Atualizar o último conf para comparação na próxima iteração
        last_conf = conf.data
    
    print("PWM Value: ", pwm_value)

def listener():	
	#Inicializano o no subscriber
	#anonymous=True significa que um numero aleatorio e adicionado ao nome do no subscriber
	rospy.init_node('client_node')#, anonymous=True)
	#Aqui nos inscrevemos, especificando o nome do topico, o tipo da msg que ira receber, e o nome da funcao callback
	rospy.Subscriber('conf_topic', Float64, callback_conf)

	#aqui nos "spin" o codigo, significa que vamos executar ele infinitamente, ate ser precionado ctrl+c
	rospy.spin()

if __name__ == "__main__":
    # Chama a funcao que inicializa o subscriber
    try:
        listener()
    except rospy.ROSInterruptException:
        pass

    

