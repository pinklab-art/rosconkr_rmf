## Ubuntu 24.04 Native 설치 
* 이 문서에서는 Ubuntu 24.04 기반 PC에 OpenRMF를 설치하는 방법을 안내합니다.
* 또한 rmf-web, rmf_demos, 실습 코드 등 워크숍에 필요한 모든 것을 pc에 설치합니다.

### 1) ROS 2 Jazzy 설치
OpenRMF 구동을 위해 ROS 2 Jazzy 버전이 필요합니다.
- 설치 방법: [ROS2 Jazzy 설치 가이드](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

### 2) OpenRMF 설치
OpenRMF 바이너리는 패키지 매니저를 통해 설치합니다.

* 종속성 설치
```bash
sudo apt update && sudo apt install ros-dev-tools -y
sudo rosdep init # run if first time using rosdep.
rosdep update
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default
```

* Open RMF 설치 
```
sudo apt update && sudo apt install ros-jazzy-rmf-dev
```

### 3) rmf_demos 데모코드 설치 
```bash
mkdir ~/rmf_ws/src -p
cd ~/rmf_ws/src
git clone https://github.com/open-rmf/rmf_demos.git -b jazzy
cd ~/rmf_ws
colcon build
```

### 4) 실습 코드 설치 
```bash
cd ~/rmf_ws/src
git clone https://github.com/pinklab-art/rosconkr_rmf.git
cd ~/rmf_ws
colcon build
```

### 5) rmf-web 설치 
* pnpm 설치
```bash
curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash -
sudo apt install -y nodejs python3-pip python3-venv
curl -fsSL https://get.pnpm.io/install.sh | bash -
source ~/.bashrc
pnpm env use --global lts
```

* rmf-web 설치 
```
cd ~
git clone https://github.com/open-rmf/rmf-web.git -b jazzy
cd rmf-web
pnpm install
```

### 최종 설치 구조
```bash
~/rmf_ws/
├── src/
│   ├── rmf_demos/
│   └── rosconkr_rmf/
├── build/
├── install/
└── log/

~/rmf-web/
├── node_modules/
├── dist/
├── packages/
├── ...
└── package.json
```

---

- 상세 설치 방법:
    * [OpenRMF GitHub](https://github.com/open-rmf/rmf)
    * [rmf-web GitHub](https://github.com/open-rmf/rmf-web)
    * [rmf_demos GitHub](https://github.com/open-rmf/rmf_demos)
    * [rosconkr_rmf GitHub](https://github.com/pinklab-art/rosconkr_rmf)