class Button {
  String label;
  float x;    // top left corner x position
  float y;    // top left corner y position
  float w;    // width of button
  float h;    // height of button
  char l;
  boolean rectOver;
  
  Button(String labelB, float xpos, float ypos, float widthB, float heightB, char letter) {
    label = labelB;
    x = xpos;
    y = ypos;
    w = widthB;
    h = heightB;
    l = letter;
    rectOver = false;
  }
  
  void Draw() {
    fill(218);
    stroke(141);
    rect(x, y, w, h, 10);
    textAlign(CENTER, CENTER);
    fill(0);
    text(label, x + (w / 2), y + (h / 2));
  }
  
  void checkLoc(){
    if(MouseIsOver() == true){
      if(rectOver == true){
        rectOver = false;
      }
      else{
        rectOver = true;
      }
      if(rectOver == true){
          sendValue();
      }
    }
  }
  
  
  boolean MouseIsOver() {
    if (mouseX > x && mouseX < (x + w) && mouseY > y && mouseY < (y + h)) {
      return true;
    }
    return false;
  }
  
  void sendValue(){
     label = "newloc";
     Draw();
  }
}
Button buttonA;
Button buttonB;
Button buttonC;
Button buttonD;
Button buttonE;
Button buttonF;
Button buttonG;
Button buttonH;
Button buttonAHH;

void setup() {
  size(1000, 1000);
  String a = "Forwards";
  buttonA = new Button(a, 450, 500, 100, 100, 'w');
  buttonA.Draw(); //<>//
  String b = "Stop";
  buttonB = new Button(b, 450, 850, 100, 100,'s');
  buttonB.Draw();
  String c = "Right";
  buttonC = new Button(c, 600, 700, 100, 100,'r');
  buttonC.Draw();
  String d = "Left";
  buttonD = new Button(d, 300, 700, 100, 100,'l');
  buttonD.Draw();
  String e = "Backwards";
  buttonE = new Button(e, 450, 700, 100, 100,'b');
  buttonE.Draw();
  String f = "Home";
  buttonF = new Button(f, 850, 850, 100, 100,'h');
  buttonF.Draw();
  String g = "Clear";
  buttonG = new Button(g, 50, 850, 100, 100,'h');
  buttonG.Draw();
  //ellipseMode(CENTER);
}

void draw(){
  if(mousePressed){
     
   update(buttonA);
   update(buttonB);
   update(buttonC);
   update(buttonD);
   update(buttonE);
   update(buttonF);
   update(buttonG);
 }
}

void update(Button button) {
  button.checkLoc();
}

void mousePressed(){
  String ahh = "ahh"; //<>//
  buttonAHH = new Button(ahh, 250, 250, 100, 100,'h');
    boolean temp = buttonA.MouseIsOver(); //<>//
    if(temp){
      buttonA.sendValue();
    }
}
