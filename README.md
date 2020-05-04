Esse repositório é fruto do meu Trabalho de Conclusão de Curso em Engenharia de Computação.

Links para visualizar o projeto sendo executado em ambiente de simulação:

- https://youtu.be/N_OyoeiHZF8

- https://youtu.be/WKHKqFszIp4

- https://youtu.be/XxSGK61x3ms

O trabalho completo pode ser lido no arquivo "TCC - Josue Araujo.pdf" que se encontra nesse repositório.


Os seguintes passos devem ser seguidos para configurar o ambinete e executar o projeto:

1º - Criar uma pasta que será o seu catkin workspace 

	No terminal executar os seguintes comandos:
	$ mkdir -p ~/catkin_ws/src

	Mais informações sobre catkin workspaces podem ser obtidos na Wiki do ROS:
	http://wiki.ros.org/catkin/Tutorials/create_a_workspace
	

2º - Clonar esse repositório na pasta src do seu workspace que acabou de ser criada.

	$ cd ~/catkin_ws/src
	$ git clone https://github.com/josuearaujo/multi_robot_sim.git

3º - Voltar para a pasta raiz do seu workspace e executar o catkin_make:

	$ cd ~/catkin_ws/
	$ catkin_make

4º - Nesse momento o workspace já está criado, mas para torná-lo conhecido no terminal, deve-se executar o seguinte comando:
	
	$ source ~/catkin_ws/devel/setup.bash
	
5º - Agora você já pode executar os comandos ROS utilizando o namespace do projeto:
	O seguinte comando, por exemplo, o levará direto para a pasta do projeto.

	$ roscd multi_robot_sim

6º - Para não ser necessário realizar o passo 4 sempre que abrir um novo terminal, é conveniente colocar esse comando no bashrc. Além disso, a configuração do sensor que será utilizado também deverá ser exportada:

	Adicionar no final do seu arquivo ~/.bashrc as seguintes linhas:

	source ~/catkin_ws/devel/setup.bash
	export TURTLEBOT_3D_SENSOR="kinect"


Com o ambiente configurado, precisa-se iniciar os seguintes módulos do sistema:
1º - Instanciar o ambiente de simulação, robos, etc:

	$ roslaunch multi_robot_sim robots_gazebo_rviz.launch 

2º - Servidor:

	$ rosrun multi_robot_sim productor_consumer.py

3º - Máquinas de estado dos Robôs:

	$ rosrun multi_robot_sim states_robot1.py
	$ rosrun multi_robot_sim states_robot2.py

4º - Cliente (Simulador de pedidos de café):

	$ rosrun multi_robot_sim client.py


A cada ENTER que for dado no terminal do cliente, um pedido de café será enviado. Será possível visualizar no Gazebo e no Rviz (iniciados no passo 1) o robô se locomovendo até o ponto de destino.


