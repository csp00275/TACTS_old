import processing.serial.*;

Serial arduino;  // 시리얼 통신을 위한 Serial 객체
int[] dataPoints1 = new int[100];  // 데이터 포인트 1을 저장할 배열
int[] dataPoints2 = new int[100];  // 데이터 포인트 2를 저장할 배열
int[] dataPoints3 = new int[100];  // 데이터 포인트 3을 저장할 배열
int[] dataPoints4 = new int[100];  // 데이터 포인트 4를 저장할 배열
int[] dataPoints5 = new int[100];  // 데이터 포인트 4를 저장할 배열

int dataIndex = 0;  // 데이터 배열의 인덱스
int x = 0;  // X 축 값 (데이터 개수)

void setup() {
  size(800, 400);  // 그래픽 윈도우의 크기 설정
  arduino = new Serial(this, "COM13", 115200);  // 아두이노와의 시리얼 통신 설정 (포트와 통신 속도)
  arduino.bufferUntil('\n');  // 개행 문자까지 데이터를 버퍼에 저장
}

void draw() {
  background(255);  // 배경을 흰색으로 설정
  drawGrid();  // 그리드 그리기
  drawGraph(dataPoints1, color(255, 0, 0), "Data Value 1");  // 데이터 포인트 1 그래프 그리기 (빨간색)
  drawGraph(dataPoints2, color(0, 255, 0), "Data Value 2");  // 데이터 포인트 2 그래프 그리기 (초록색)
  drawGraph(dataPoints3, color(0, 0, 255), "Data Value 3");  // 데이터 포인트 3 그래프 그리기 (파란색)
  drawGraph(dataPoints4, color(255, 0, 255), "Data Value 4");  // 데이터 포인트 4 그래프 그리기 (자홍색)
  drawGraph(dataPoints4, color(0,255, 255), "Data Value 4");  // 데이터 포인트 4 그래프 그리기 (자홍색)

  drawLegend();  // 레전드 그리기
}

void drawGrid() {
  // 그리드 선 색상
  stroke(200);

  // 가로 그리드
  for (int i = 1; i < 10; i++) {
    float yPos = map(i, 0, 10, height - 50, 50);  // Y 좌표 계산
    line(50, yPos, width - 50, yPos);
  }

  // 세로 그리드
  for (int i = 1; i < x; i++) {
    float xPos = map(i, 0, x - 1, 50, width - 50);  // X 좌표 계산
    line(xPos, 50, xPos, height - 50);
  }
}

void drawGraph(int[] dataPoints, color lineColor, String legend) {
  // X축
  stroke(0);
  line(50, height - 50, width - 50, height - 50);

  // Y축
  line(50, 50, 50, height - 50);

  // 데이터 그래프
  noFill();
  beginShape();
  stroke(lineColor);
  for (int i = 0; i < x; i++) {
    float xPos = map(i, 0, x - 1, 50, width - 50);  // X 좌표 계산
    float yPos = map(dataPoints[i], 0, 1023, height - 50, 50);  // Y 좌표 계산
    vertex(xPos, yPos);
  }
  endShape();
}

void drawLegend() {
  // 레전드 텍스트
  fill(0);
  textSize(14);
  textAlign(CENTER, CENTER);

  // 데이터 갯수 레전드
  text("Data Count", width / 2, height - 10);

  // 데이터 값 레전드
  int legend1 = 650;
  int startY = 100;
  int gap = 20;
  pushMatrix();
  translate(20, height / 2);
  fill(255, 0, 0);
  text("Data Value 1", legend1, -startY + gap);
  fill(0, 255, 0);
  text("Data Value 2", legend1, -startY + 2*gap);
  fill(0, 0, 255);
  text("Data Value 3", legend1, -startY + 3*gap);
  fill(255, 0, 255);
  text("Data Value 4", legend1, -startY + 4*gap);
  fill(0, 255, 255);
  text("Data Value 4", legend1, -startY + 5*gap);
  popMatrix();
}

void serialEvent(Serial port) {
  String data = port.readStringUntil('\n');  // 아두이노에서 받은 데이터를 읽음
  if (data != null) {
    data = data.trim();  // 데이터의 앞뒤 공백 제거
    String[] values = data.split(" ");  // 데이터를 공백을 기준으로 분리

    if (values.length >= 4) {
      int value1 = int(values[0]);  // 데이터1을 정수형으로 변환
      int value2 = int(values[1]);  // 데이터2를 정수형으로 변환
      int value3 = int(values[2]);  // 데이터3을 정수형으로 변환
      int value4 = int(values[3]);  // 데이터4를 정수형으로 변환
      int value5 = int(values[4]);  // 데이터4를 정수형으로 변환


      // 데이터 배열에 추가
      dataPoints1[dataIndex] = value1;
      dataPoints2[dataIndex] = value2;
      dataPoints3[dataIndex] = value3;
      dataPoints4[dataIndex] = value4;
      dataPoints5[dataIndex] = value5;
      dataIndex = (dataIndex + 1) % dataPoints1.length;

      // X 축 값 증가
      x = min(x + 1, dataPoints1.length);
    }
  }
}
