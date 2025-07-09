# WORK\_ROS2\_WS

Um espaço de trabalho para desenvolvimento com ROS 2.

## Descrição

Este repositório contém um espaço de trabalho (workspace) para o desenvolvimento de projetos com o ROS 2 (Robot Operating System 2).

## Pré-requisitos

Antes de começar, certifique-se de que você tem os seguintes pré-requisitos instalados no seu sistema:

  * **Sistema Operacional:** Ubuntu 24.04 (Noble Numbat)
  * **ROS 2:** Jazzy Jalisco

## Instalação

Siga os passos abaixo para configurar o ambiente e o repositório.

### 1\. Instalação do ROS 2 Jazzy Jalisco

Se você ainda não tem o ROS 2 Jazzy instalado, siga estas etapas. Para mais detalhes, consulte a [documentação oficial do ROS 2](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).

**a. Configure suas fontes de repositório:**

Primeiro, adicione o repositório do ROS 2 ao seu sistema.

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

**b. Adicione a chave GPG do ROS 2:**

```bash
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

**c. Adicione o repositório às suas fontes:**

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

**d. Instale o ROS 2:**

Atualize seus pacotes e instale a versão Desktop completa do ROS 2 Jazzy:

```bash
sudo apt update
sudo apt upgrade
sudo apt install ros-jazzy-desktop
```

### 2\. Clone e compile o workspace

Com o ROS 2 instalado, clone este repositório e compile o workspace.

**a. Clone o repositório:**

```bash
git clone https://github.com/UnbDroid/WORK_ROS2_WS.git
cd WORK_ROS2_WS
```

**b. Compile o workspace:**

```bash
# Carregue o ambiente do ROS 2
source /opt/ros/jazzy/setup.bash

# Compile o workspace
colcon build
```

## Como Usar

Após a compilação, carregue o ambiente do seu workspace para poder executar os pacotes contidos nele.

```bash
source install/setup.bash
```

Agora você pode usar os comandos do ROS 2 para interagir com os pacotes deste workspace. Por exemplo, para executar um nó (substitua `nome_do_pacote` e `nome_do_executavel` pelos nomes corretos):

```bash
ros2 launch trekking main.launch.py
```

## Contribuição

Contribuições são bem-vindas\! Se você deseja contribuir com este projeto, por favor, siga estas etapas:

1.  Faça um "fork" do repositório.
2.  Crie uma nova "branch" para a sua feature (`git checkout -b feature/nova-feature`).
3.  Faça o "commit" das suas alterações (`git commit -am 'Adiciona nova feature'`).
4.  Faça o "push" para a sua branch (`git push origin feature/nova-feature`).
5.  Abra um "Pull Request".
