# object_detection
object detection for detecting wall and dynamic&amp;static obstacles

## TODO  
# 의존성 설치  
cd object_detector  
pip install -r requirements.txt  

# cordinate.py import  
object_detector/src/tracker.py 내의   
sys.path.append('#해당 directory#/cordinate/src') -> directory 수정  

# 패키지 빌드  
colcon build #object_detector  
cd ../cordinate  
colcon build #cordinate   

# frenet_converter & globalpath_loader 실행  
cd cordinate  
source install/setup.bash  
ros2 run cordinate.cpp  

# object detector & tracker 실행  
cd object_detector  
source install/setup.bash  
ros2 launch object_detect_launch.py  

