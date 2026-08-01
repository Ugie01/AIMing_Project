prepare_dataset.py
: raw의 이미지에서 드론과 배경을 분리해 드론은 drone, 배경은 bg에 저장

ai_test.py
: bg와 drone을 합성(드론위치 랜덤)해서 AI 모델에 집어넣고 결과물을 AI_Test_Result에 저장
==> AI 성능 확인 및 시각화 용

poc_test.py
: bg와 drone을 합성(드론위치 랜덤)해서 화면에 출력 (bg+drone 1세트당 10번씩)
입력 = 1: 적중, 2: 미적중, 3: 미검출, 4: 오검출, R: 무효 ESC: 종료
results에 저장 (최초 Fire 시점에 데이터 시간과 같이 저장)