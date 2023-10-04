import processing.serial.*;

Serial arduino;
final int NUM_DATA_POINTS = 6;  // 이 값을 변경하여 데이터의 수를 조절하세요.
int[][] dataPoints = new int[NUM_DATA_POINTS][100];
int dataIndex = 0;
int x = 0;
color[] colors = {color(255, 0, 0), color(0, 255, 0), color(0, 0, 255),
color(128, 0, 128), color(0, 128, 128), color(0, 0, 0)};


final int MOVING_WINDOW_SIZE = 20;

float movingMean;
float movingStdDev;

// Y 축의 최대, 최소 값을 조절하기 위한 상수
final int Y_MIN = 40;
final int Y_MAX = 60;  // 이 값을 변경하여 Y축의 최대 값을 조절하세요.


void setup() {
  size(1000, 1000);  // 그래픽 윈도우의 크기를 수정
  arduino = new Serial(this, "COM11", 115200);
  arduino.bufferUntil('\n');
}
void draw() {
  background(255);

  // 위쪽 그래프 그리기
  pushMatrix();
  translate(0, 50);
  for (int i = 0; i < NUM_DATA_POINTS; i++) {
    drawGraph(dataPoints[i], colors[i], "Sensor " + (i + 1), 0, 300);
  }
  drawYAxisLabelsForTopGraph(300);  // 위쪽 그래프의 Y 라벨을 그립니다.
  popMatrix();

  // 아래쪽 그래프 그리기
  pushMatrix();
  translate(0, 600);
  for (int i = 0; i < NUM_DATA_POINTS; i++) {
    calculateMovingStatistics(dataPoints[i]);  // 각 센서 데이터의 통계 계산
    drawPDF(0, 300, colors[i]);  // PDF 그리기 함수에 color 추가
  }
  drawXAxisLabelsForPDF(300);
  popMatrix();
  
  drawLegend();
}

void calculateMovingStatistics(int[] data) {
  int endIndex = min(dataIndex, data.length);
  int startIndex = max(0, endIndex - MOVING_WINDOW_SIZE);
  
  int sum = 0;
  for (int i = startIndex; i < endIndex; i++) {
    sum += data[i];
  }
  movingMean = float(sum) / (endIndex - startIndex);

  float variance = 0;
  for (int i = startIndex; i < endIndex; i++) {
    variance += pow(data[i] - movingMean, 2);
  }
  variance /= (endIndex - startIndex);
  movingStdDev = sqrt(variance);
}

void drawPDF(float startY, float graphHeight, color lineColor) {
  stroke(lineColor);
  noFill();
  beginShape();
  for (float i = Y_MIN; i <= Y_MAX; i += 0.1) {
    float pdfValue = exp(-0.5 * pow((i - movingMean) / movingStdDev, 2)) / (movingStdDev * sqrt(TWO_PI));
    float y = map(pdfValue, 0, 0.5, startY + graphHeight, startY);  // PDF 값이 0에서 0.5 사이라고 가정
    vertex(map(i, Y_MIN, Y_MAX, 50, width - 50), y);
  }
  endShape();
}

void drawXAxisLabelsForPDF(float graphHeight) {
  int numOfLabels = 10;
  textSize(10);
  textAlign(CENTER, TOP);
  fill(0);
  
  for (int i = 0; i <= numOfLabels; i++) {
    float percent = i / float(numOfLabels);
    float xPos = lerp(50, width - 50, percent);
    float labelValue = lerp(Y_MIN, Y_MAX, percent);
    text(nf(labelValue, 0, 1), xPos, graphHeight + 5); // nf 함수로 소수점 한 자리까지 표시
  }
}


void drawYAxisLabelsForTopGraph(float graphHeight) {
  int numOfLabels = 10;
  textSize(10);
  textAlign(RIGHT, CENTER);
  fill(0);
  
  for (int i = 0; i <= numOfLabels; i++) {
    float percent = i / float(numOfLabels);
    float yPos = lerp(graphHeight, 0, percent);
    int label = int(lerp(Y_MIN, Y_MAX, percent));
    text(label, 40, yPos);
  }
}
//void drawGrid() {
//  // 그리드 선 색상
//  stroke(200);

//  // 가로 그리드
//  for (int i = 1; i < 10; i++) {
//    float yPos = map(i, 0, 10, height - 50, 50);  // Y 좌표 계산
//    line(50, yPos, width - 50, yPos);
//  }

//  // 세로 그리드
//  for (int i = 1; i < x; i++) {
//    float xPos = map(i, 0, x - 1, 50, width - 50);  // X 좌표 계산
//    line(xPos, 50, xPos, height - 50);
//  }
//}

void drawGraph(int[] dataPoints, color lineColor, String legend, float startY, float graphHeight) {
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
    float xPos = map(i, 0, x - 1, 50, width - 50);
    float yPos = map(dataPoints[i], Y_MIN, Y_MAX, startY + graphHeight, startY);
    vertex(xPos, yPos);
  }
  endShape();
}
// 레전드 그리기 함수에서 위치를 조정합니다.
void drawLegend() {
  int legendX = 800;
  int startY = 0;
  int gap = 30;
  String[] legendNames = {"Raw", "MAF5", "MMF5", "KF1", "KF2", "KF3"};
  
  textSize(20); // 글자 크기 변경
  
  for (int i = 0; i < NUM_DATA_POINTS; i++) {
    fill(colors[i]);
    rect(legendX, startY + i * gap, 20, 20); // 색상 박스 크기 변경
    fill(0);
    text(legendNames[i], legendX +55, startY + i * gap);
  }
}



void serialEvent(Serial port) {
  String data = port.readStringUntil('\n');
  if (data != null) {
    data = data.trim();
    String[] values = data.split(" ");
    
    if (values.length >= NUM_DATA_POINTS) {
      for (int i = 0; i < NUM_DATA_POINTS; i++) {
        int value = int(values[i]);
        dataPoints[i][dataIndex] = value;
      }
      dataIndex = (dataIndex + 1) % dataPoints[0].length;
      x = min(x + 1, dataPoints[0].length);
    }
  }
}
