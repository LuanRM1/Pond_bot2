# README

Este projeto permite a teleoperação de um robô usando ROS 2 (Humble) e uma interface gráfica desenvolvida com Pygame. O robô pode ser controlado utilizando o teclado e botões na interface gráfica, e exibe informações em tempo real sobre a velocidade do robô e latência das mensagens recebidas.

Video de demonstração [video](https://drive.google.com/file/d/1sA_-aW7vClSp4_rA2u7rdsUSDYBuygdB/view?usp=sharing)

### Pré-requisitos

- Python 3
- ROS 2 Humble
- Bibliotecas Python:
  - `rclpy`
  - `geometry_msgs`
  - `sensor_msgs`
  - `std_msgs`
  - `std_srvs`
  - `pygame`
  - `cv2`
  - `numpy`
  - `Pillow`

### Instalação

1. **Instalar ROS 2 Humble**:
   
   Siga as instruções de instalação no [site oficial do ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html).

2. **Instalar dependências Python**:
   
   ```bash
   pip install pygame opencv-python numpy Pillow rclpy
   ```

   caso tenha problema com o rclpy utilize esse repo: [rclpy](https://github.com/ros2/rclpy)

3. **Clonar o repositório**:
   
   Clone este repositório na sua máquina:

   ```bash
   git clone https://github.com/LuanRM1/Pond_bot2.git
   cd Pond_bot2
   ```
   No robo deixe apenas o main_ws, e no computador deixa apenas o visor.
   

5. **Configurar o ambiente ROS 2**:
   
   No terminal, configure o ambiente do ROS 2:

   ```bash
   source /opt/ros/humble/setup.bash
   ```

### Executando o Projeto

1. **Iniciar os nós ROS 2**:
   No robô:
   ```bash
   cd main_ws
   source install/local_setup.bash
   cd launch
   ros2 launch launch.py
   ```
   No computador:
   Execute o arquivo `main.py` com Python 3:

   ```bash
   cd visor
   python3 main.py
   ```

3. **Interagir com a interface**:
   
   - Use as teclas `W`, `A`, `S`, `D` para mover o robô para frente, esquerda, trás e direita, respectivamente.
   - Pressione `SPACE` para parar o robô.
   - Pressione `Q` para sair do programa.
   - Pressione `B` para acionar o botão de emergência (Kill Switch).
   - Clique nos botões da interface gráfica para mover o robô ou acionar o Kill Switch.

### Descrição dos Arquivos

- `main.py`: Script principal que configura a interface gráfica e os nós ROS 2.
- `arrow_up.png`, `arrow_down.png`, `arrow_left.png`, `arrow_right.png`: Imagens das setas utilizadas nos botões da interface gráfica.

### Estrutura do Código

O código é composto pelos seguintes componentes principais:

- **RobotController**: Classe responsável pelo controle do robô, incluindo movimentação, parada de emergência e callbacks do LiDAR.
- **Listener**: Classe responsável por receber e exibir imagens transmitidas.
- **init_ros_nodes**: Função que inicializa os nós ROS 2.
- **main**: Função principal que configura a interface gráfica e gerencia a interação do usuário.
