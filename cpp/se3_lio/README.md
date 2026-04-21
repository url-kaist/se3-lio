

# Build shared library for SE3-LIO
```bash
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install
make -j$(nproc)
make install
```

# TODO
<!-- checkpoint -->
[] 1. URL LiDAR SLAM에서 내 형태에 맞게 이식
[] 2. Voxel map 구현
[] 3. IMU propagation 간소화
[] 4. ros1, ros2 안정화
[] 5. python 바인딩
[] 6. 여러 데이터셋에서 테스트 후 yaml 파일 제작, 영상 제작
[] 7. 빌드 및 설치 방법 정리
[] 8. Project page 작성
[] 9. 특허준비