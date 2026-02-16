# RoboSim 빌드 수정 로그

**날짜**: 2026년 2월 16일  
**환경**: Ubuntu 24.04, conda bob_env1, CMake 3.28.3, GCC 13.3.0

## 발생한 문제와 해결 방법

### 1. cmake/dependencies.cmake 파일 누락

**문제**:
```
CMake Error at CMakeLists.txt:16 (include):
  include could not find requested file:
    cmake/dependencies.cmake
```

**원인**: 프로젝트에 `cmake/` 디렉토리와 `dependencies.cmake` 파일이 존재하지 않음

**해결**:
- `cmake/dependencies.cmake` 파일 생성
- FetchContent를 사용하여 다음 의존성을 자동으로 다운로드하도록 구성:
  - Eigen3 (3.4.0)
  - Bullet3 (3.25)
  - yaml-cpp (0.8.0)
  - spdlog (v1.12.0)
  - GLFW (3.3.8) - GUI 빌드 시
  - ImGui (v1.89.9) - GUI 빌드 시

---

### 2. ImGui 버전 태그 오류

**문제**:
```
fatal: invalid reference: v1.90.0
CMake Error: Build step for imgui failed
```

**원인**: ImGui 저장소에 `v1.90.0` 태그가 존재하지 않음

**해결**:
- ImGui 버전을 `v1.89.9`로 변경
- 위치: `cmake/dependencies.cmake` 94번째 줄

---

### 3. Eigen 표현식 타입 불일치 오류

**문제**:
```cpp
error: operands to '?:' have different types
ctx.target_velocity = (dist > 1e-6) ? (diff / dist) * 1.0 : Eigen::Vector3d::Zero();
```

**원인**: 
- 삼항 연산자의 두 분기가 서로 다른 Eigen 표현식 타입을 반환
- 왼쪽: `CwiseBinaryOp<...>` (지연 평가 표현식)
- 오른쪽: `CwiseNullaryOp<...>` (상수 표현식)

**해결**:
- 삼항 연산자를 if-else 문으로 변경
- 위치: `include/robosim/task/tasks/all_tasks.h` 110번째 줄

**수정 전**:
```cpp
ctx.target_velocity = (dist > 1e-6) ? (diff / dist) * 1.0 : Eigen::Vector3d::Zero();
```

**수정 후**:
```cpp
if (dist > 1e-6) {
    ctx.target_velocity = (diff / dist) * 1.0;
} else {
    ctx.target_velocity = Eigen::Vector3d::Zero();
}
```

---

### 4. Bullet Physics API 파라미터 불일치

**문제**:
```cpp
error: no matching function for call to 'btMultiBody::setupPrismatic(...)'
note: candidate expects 9 arguments, 8 provided
```

**원인**: 
- `setupPrismatic()` 메서드 호출 시 9번째 파라미터 `pivot_to_com` 누락
- Bullet Physics 3.25에서 API가 변경됨

**해결**:
- `pivot_to_com` 파라미터 추가
- 위치: `src/physics/bullet_world.cpp` 143-145번째 줄

**수정 전**:
```cpp
mb->setupPrismatic(link_idx, link_mass, link_inertia,
                   parent_idx, btQuaternion::getIdentity(),
                   joint_axis, parent_to_pivot, !can_sleep);
```

**수정 후**:
```cpp
mb->setupPrismatic(link_idx, link_mass, link_inertia,
                   parent_idx, btQuaternion::getIdentity(),
                   joint_axis, parent_to_pivot, pivot_to_com, !can_sleep);
```

---

### 5. World 클래스 구현 파일 누락

**문제**:
```
undefined reference to `robosim::core::World::World(robosim::core::WorldConfig const&)'
```

**원인**: 
- `World` 클래스의 헤더 파일(`world.h`)은 존재하지만 구현 파일이 없음
- 생성자가 정의되지 않음

**해결**:
- `src/core/world.cpp` 파일 생성
- 생성자 구현 추가

```cpp
#include "robosim/core/world.h"

namespace robosim::core {

World::World(const WorldConfig& config) : config_(config), sim_time_(0.0) {
    // Base constructor implementation
}

} // namespace robosim::core
```

---

### 6. GLAD OpenGL 로더 문제 (복합 이슈)

**문제**:
```cpp
error: 'GLADloadproc' was not declared in this scope
error: 'gladLoadGLLoader' was not declared in this scope
error: 'GLVersion' was not declared in this scope
error: 'GLuint64EXT' has not been declared  // 시스템 GL 헤더와 충돌
```

**원인**: 
- GLAD 원본 파일(`glad.h`, `glad.c`, `khrplatform.h`)이 모두 **0바이트 빈 파일**로 포함되어 있었음
- 렌더링 소스 파일에 GLAD include가 누락되어 있었음

이 문제는 여러 하위 이슈로 분리되며 순차적으로 해결함:

#### 6-1. GLAD 소스 파일 재생성

**원인**: `third_party/glad/` 내 파일들이 비어있어 OpenGL 함수를 로드할 수 없었음

**해결**:
- `pip install glad2`로 GLAD2 생성 도구 설치
- 확장 기능 없이 OpenGL 3.3 Core 프로파일만 포함하도록 재생성:
  ```bash
  python -m glad --api="gl:core=3.3" --extensions="" --out-path=third_party/glad c
  ```
- **주의**: GLAD2는 `glad.h`/`glad.c`가 아닌 **`gl.h`/`gl.c`**를 생성함 (GLAD1과 파일명이 다름)
- 생성 결과: `gl.h` (110KB), `gl.c` (58KB), `khrplatform.h` (11KB)

#### 6-2. 모든 확장 기능 포함으로 인한 타입 충돌

**원인**: 첫 번째 생성 시 `--extensions=""` 옵션 없이 실행하여 622개 확장 기능이 포함됨. 시스템 GL 헤더(`/usr/include/GL/glext.h`)와 `GLuint64EXT`, `GLintptrARB` 등 타입 정의가 충돌함

**해결**: `--extensions=""` 옵션으로 확장 기능 0개 상태로 재생성

#### 6-3. GLAD 헤더 경로 불일치

**원인**: 
- 소스/헤더 파일에서 `#include <glad/glad.h>` 사용 (GLAD1 형식)
- GLAD2가 생성한 파일명은 `glad/gl.h`

**수정된 파일**:

| 파일 | 수정 내용 |
|------|-----------|
| `include/robosim/render/mesh.h` | `<glad/glad.h>` → `<glad/gl.h>` |
| `include/robosim/render/shader.h` | `<glad/glad.h>` → `<glad/gl.h>` |
| `src/render/opengl_renderer.cpp` | `<glad/gl.h>` 최상단 include 추가 |
| `src/render/gui.cpp` | `<glad/gl.h>` 최상단 include 추가 |

#### 6-4. GLAD API 호출 수정 (GLAD1 → GLAD2)

**원인**: GLAD2의 초기화 API가 GLAD1과 다름

**수정** (`src/render/opengl_renderer.cpp`):

**수정 전** (GLAD1 API):
```cpp
if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) { ... }
spdlog::info("OpenGL {}.{} initialized", GLVersion.major, GLVersion.minor);
```

**수정 후** (GLAD2 API):
```cpp
int gl_version = gladLoadGL(glfwGetProcAddress);
if (gl_version == 0) { ... }
int gl_major = GLAD_VERSION_MAJOR(gl_version);
int gl_minor = GLAD_VERSION_MINOR(gl_version);
spdlog::info("OpenGL {}.{} initialized", gl_major, gl_minor);
```

#### 6-5. CMake GLAD 소스 경로 업데이트

**수정** (`cmake/dependencies.cmake`):
```cmake
# 수정 전
set(GLAD_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/third_party/glad/src/glad.c)
# 수정 후
set(GLAD_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/third_party/glad/src/gl.c)
```

---

### 7. YAML ConfigNode 런타임 크래시 (실행 시 오류)

**문제**:
```
terminate called after throwing an instance of 'YAML::TypedBadConversion<double>'
  what():  bad conversion
Aborted (core dumped)
```
- 빌드는 성공하지만 `./robosim --headless --robot unitree_go2 --task walk_forward --steps 10000` 실행 시 크래시 발생

**원인**:
- `ConfigNode::resolve()` 메서드에서 빈/Undefined YAML 노드에 `operator[]`를 호출하면, yaml-cpp 0.8.0이 묵시적으로 Map을 생성하여 잘못된 노드를 반환
- 이 노드가 `get<double>("timestep", 0.002)` 호출 시 존재하는 것처럼 평가되어 `.as<double>()`을 시도하지만 실제 값이 없으므로 예외 발생
- `ConfigNode::child()` 메서드도 빈 노드를 체크하지 않고 그대로 전달

**해결** (`src/config/config_node.cpp`):

**resolve() 수정 전**:
```cpp
YAML::Node ConfigNode::resolve(const std::string& dotted_key) const {
    std::istringstream iss(dotted_key);
    std::string token;
    YAML::Node current = node_;
    while (std::getline(iss, token, '.')) {
        if (!current.IsMap() || !current[token]) {
            return YAML::Node();
        }
        current = current[token];
    }
    return current;
}
```

**resolve() 수정 후**:
```cpp
YAML::Node ConfigNode::resolve(const std::string& dotted_key) const {
    if (!node_ || !node_.IsDefined()) {
        return YAML::Node();
    }
    std::istringstream iss(dotted_key);
    std::string token;
    YAML::Node current = YAML::Clone(node_);
    while (std::getline(iss, token, '.')) {
        if (!current.IsMap() || !current[token] || !current[token].IsDefined()) {
            return YAML::Node();
        }
        current = current[token];
    }
    return current;
}
```

**child() 수정 전**:
```cpp
ConfigNode ConfigNode::child(const std::string& key) const {
    auto node = resolve(key);
    return ConfigNode(node, path_ + "." + key);
}
```

**child() 수정 후**:
```cpp
ConfigNode ConfigNode::child(const std::string& key) const {
    if (!node_ || !node_.IsDefined() || !node_.IsMap()) {
        return ConfigNode();
    }
    auto node = resolve(key);
    return ConfigNode(node, path_ + "." + key);
}
```

**핵심 수정 포인트**:
1. `resolve()`: 빈/Undefined 노드 사전 체크 + `YAML::Clone()`으로 원본 노드 변형 방지
2. `child()`: 빈/Undefined/비-Map 노드에 대해 빈 `ConfigNode` 반환
3. `IsDefined()` 체크 추가로 yaml-cpp의 묵시적 노드 생성 방지

---

## 최종 빌드 및 실행 결과: ✅ 성공 (2026년 2월 16일)

### 빌드
```
[100%] Built target basic_sim
[100%] Built target robosim_render
[100%] Built target robosim
```

### 실행
```
./robosim --headless --robot unitree_go2 --task walk_forward --steps 10000
```
```
[info] Loading config: config/default.yaml
[info] Bullet physics world initialized
[info] Robot 'unitree_go2' created with 12 joints, 17 links
[info] Simulator initialized: robot='unitree_go2', dt=0.002, decimation=4
[info] Observation dim: 49, Action dim: 12
[info] Episode 1 ended at step 999 | Reward: -14889.612
...
[info] Episode 10 ended at step 9999 | Reward: -12913.194
[info] Headless simulation complete. 10 episodes.
```

### 수정 완료 전체 목록
- ✅ `cmake/dependencies.cmake` 생성 (FetchContent로 의존성 자동 다운로드)
- ✅ ImGui 버전 태그 수정 (v1.90.0 → v1.89.9)
- ✅ Eigen 타입 불일치 수정 (삼항 연산자 → if-else)
- ✅ Bullet Physics API 파라미터 수정 (setupPrismatic에 pivot_to_com 추가)
- ✅ `src/core/world.cpp` 생성 (World 클래스 생성자 구현)
- ✅ GLAD 재생성 (core only, 확장 0개, glad2 사용)
- ✅ GLAD 헤더 경로 수정 (`glad/glad.h` → `glad/gl.h`) — 4개 파일
- ✅ GLAD API 호출 수정 (GLAD1 → GLAD2 호환)
- ✅ CMake GLAD 소스 경로 수정 (`glad.c` → `gl.c`)
- ✅ ConfigNode YAML 파싱 크래시 수정 (resolve/child 안전 체크)

---

## 참고 사항

### 빌드 명령어
```bash
cd build
rm -rf *  # 클린 빌드 권장
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

### 의존성 요구사항
- CMake 3.20+
- GCC 9+ 또는 Clang 10+
- Python 3.8+ (GLAD 생성용)
- OpenGL 개발 라이브러리 (`libgl1-mesa-dev` 등)

### 수정된 파일 전체 목록
```
Robosim-main/
├── cmake/
│   └── dependencies.cmake              [새로 생성]
├── third_party/glad/
│   ├── include/
│   │   ├── glad/gl.h                   [재생성 - GLAD2]
│   │   └── KHR/khrplatform.h           [재생성 - GLAD2]
│   └── src/gl.c                        [재생성 - GLAD2]
├── src/
│   ├── config/
│   │   └── config_node.cpp             [수정됨 - resolve/child 안전 체크]
│   ├── core/
│   │   └── world.cpp                   [새로 생성]
│   ├── physics/
│   │   └── bullet_world.cpp            [수정됨 - setupPrismatic 파라미터]
│   └── render/
│       ├── gui.cpp                     [수정됨 - glad/gl.h include 추가]
│       └── opengl_renderer.cpp         [수정됨 - GLAD2 API + include]
└── include/robosim/
    ├── render/
    │   ├── mesh.h                      [수정됨 - glad/gl.h]
    │   └── shader.h                    [수정됨 - glad/gl.h]
    └── task/tasks/
        └── all_tasks.h                 [수정됨 - Eigen if-else]
```
