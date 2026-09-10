<!-- 역할: 제공 가중치 출처·중복 기록. 인터페이스: perception.model_paths. -->
# 인식 모델

첨부 최신 ZIP의 `yolo11m-seg.pt`, `yolo-bread.pt`를 바이트 그대로 보존했다.
`yolo11m-seg.pt.1`은 SHA-256이 동일하여 중복 사본을 포함하지 않는다.

이 작업에서는 첨부 가중치를 역직렬화하거나 실제 카메라 추론에 사용하지 않았다.
따라서 빵 외 양상추·커피 용기 등의 클래스 지원 여부와 실제 인식률을 주장하지 않는다.
사용할 클래스 모델을 `perception.model_paths`, 명칭 대응을 `label_aliases`에 설정한다.
빨간 바구니 손잡이는 이 가중치 없이 HSV backend 또는 YOLO+HSV 병합으로 검출한다.

바이너리 모델 파일에는 주석을 삽입할 수 없으므로 역할과 인터페이스를 여기 기록한다.
원본 파일 해시는 `docs/ORIGINAL_SHA256.txt`를 참조한다.
