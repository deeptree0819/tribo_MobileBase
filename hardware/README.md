# Tribo 설계 자료

Tribo 로봇의 3D 설계 파일 및 3D 프린팅용 파일 모음입니다.

- [1차설계파일](./1차설계파일)
- [최종설계파일](./최종설계파일)
- [step](./step)
- [stl](./stl)
- [가공용 도면](./가공용%20도면)

## 1차설계파일

- 1차 Tribo 설계 파일입니다.
- 최종 설계 파일에서 참조하고 있는 모델들이 포함되어 있어서, **최종설계파일과 함께 다운로드**해야 정상적으로 열립니다.

## 최종설계파일

- 26년 8월 기준으로 업데이트된 Tribo 설계 파일입니다.
- 모터 커버와 LCD 모니터 거치대는 아직 최신화되어 있지 않습니다.
- 최종 어셈블 파일은 `Tribo_Trolly_withArm`, `Tribo_Trolly_withCamera`에 저장되어 있습니다.
    - **Tribo_Trolly_withArm**: Tribo에 양팔 로봇을 장착시킨 버전
    - **Tribo_Trolly_withCamera**: Tribo에 카메라만 장착시킨 버전

## step

- Tribo의 STEP 파일이 담겨져 있는 폴더입니다.

## stl

- Tribo의 3D 프린팅을 위한 STL 파일들이 담겨져 있는 폴더입니다.

### 3D 프린팅 부품 리스트

| 부품 | 설명 |
| --- | --- |
| `Bearing_Spacer_V3` | 베어링과 프로파일을 체결하기 위한 스페이서. 스페이서별로 높이를 조금 더 높인 버전들이 추가로 있습니다. |
| `camerabase_set` | Tribo에 카메라를 장착시키기 위한 베이스. `camerabase_up`, `camerabase_down`을 각각 프린트해서 조립해야 합니다. ⚠️ 출력 시 본체보다 서포터가 더 많이 출력되는 것을 확인할 수 있습니다. |
| `Gimbal_Neck` | 카메라 베이스에 연결되는 넥(neck) |
| `D435_거치대` | 넥(neck)에 D435 카메라를 고정시키기 위한 거치대 |
| `LCD_Bottom_Cover` | LCD 모니터 케이스의 후면 케이스 |
| `LCD_Upper_Cover` | LCD 모니터 케이스의 전면 케이스 |
| `LCD_Mount_20degrees_V3` | 프로파일과 LCD 케이스를 45도 각도로 연결하기 위한 거치대 |
| `Motor_Cover_LH`, `Motor_Cover_RH` (V3) | 외부로 노출된 모터를 보호하기 위한 케이스 (좌/우) |

## 가공용 도면

- 가공할 때 요청하기 위해 별도로 만든 도면 파일입니다.
