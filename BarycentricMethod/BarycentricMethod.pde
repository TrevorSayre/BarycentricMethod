//********************************************************************
//* Barycentric Method for the Point in Triangle Test                *
//*                                                                  *
//* Modeled after the flash example on Black Pawn:                   *
//* http://www.blackpawn.com/texts/pointinpoly/default.html          *
//*                                                                  *
//* Move mouse to demonstrate point in triangle detection            *
//* Click mouse to generate a new triangle                           *
//*                                                                  *
//* Trevor Sayre                                                     *
//* sayret@gmail.com                                                 *
//* November 4, 2010                                                 *
//********************************************************************

// TODO: cartesian to PVector

int rad = 12;     // radius of the point circles
int tSize = 12;   // size of the text
int lStroke = 4;  // size of the line stroke
float u, v;       // barycentric normalized coordinates
int t1x, t1y, t2x, t2y, t3x, t3y;  // triangle coordinates

void setup() {
  
  size(300,300);
  smooth();
  
  // generate the initial traingle coordinates
  generateTriangle();
  
}

void draw() {
  
  background(0);
  strokeWeight(lStroke);
  
  // check if the mouse is within the triangle boundaries
  // change the fill for the triangle depending on the result
  if(checkCollision(t1x,t1y,t2x,t2y,t3x,t3y,mouseX,mouseY)) {
    fill(212, 216, 0); // yellow
  }
  else {
    fill(45, 53, 104); // purple
  }
  
  // draw the triangle
  stroke(69,83,167);   // light purple
  triangle(t1x,t1y,t2x,t2y,t3x,t3y);
  
  // draw a line from t1 to the mouse
  stroke(255,255,255,128);  // white at 50% opacity
  line(t1x,t1y,mouseX,mouseY);
  
  // draw the barycentric vectors
  // v, t1 -> t2
  float magV = dist(t1x, t1y, t2x, t2y)*v;
  PVector V = resizeVector(t1x, t1y, t2x, t2y, magV);
  stroke(0, 255, 0);  // green
  arrow(t1x,t1y,int(V.x),int(V.y));
  // u, t1 -> t3
  float magU = dist(t1x, t1y, t3x, t3y)*u;
  PVector U = resizeVector(t1x, t1y, t3x, t3y, magU);
  stroke(255, 0, 0);  // red
  arrow(t1x,t1y,int(U.x),int(U.y));
     
  // draw an ellipse at the three triangle points 
  // and the mouse position   
  noStroke();
  fill(255,255,255);  // white
  ellipse(mouseX,mouseY,rad,rad);
  ellipse(t1x,t1y,rad,rad);
  ellipse(t2x,t2y,rad,rad);
  ellipse(t3x,t3y,rad,rad);
  
  // display the barycentric coordinate values
  fill(128,128,128);  // gray
  textSize(tSize);
  textAlign(LEFT,CENTER);
  text("u = " + u, rad, rad);
  text("v = " + v, rad, rad+tSize);
  
  // calculate locations of point lables A, B, and C
  // A location
  // use midpoint formula to find center of BC
  int a0x = (t2x+t3x)/2;
  int a0y = (t2y+t3y)/2;
  float magA = dist(a0x,a0y,t1x,t1y) + (rad*2);
  PVector A = resizeVector(a0x, a0y, t1x, t1y, magA);
  // B location
  // use midpoint formula to find center of AC
  int b0x = (t1x+t3x)/2;
  int b0y = (t1y+t3y)/2;
  float magB = dist(b0x,b0y,t2x,t2y) + (rad*2);
  PVector B = resizeVector(b0x, b0y, t2x, t2y, magB);
  // C location
  // use midpoint formula to find center of AB
  int c0x = (t1x+t2x)/2;
  int c0y = (t1y+t2y)/2;
  float magC = dist(c0x,c0y,t3x,t3y) + (rad*2);
  PVector C = resizeVector(c0x, c0y, t3x, t3y, magC);
  
  // label the triangle points
  textSize(tSize*1.5);
  textAlign(CENTER,CENTER);
  text("A",A.x,A.y);
  text("B",B.x,B.y);
  text("C",C.x,C.y);
  
}

// if the mouse is clicked, generate a new triangle
void mouseClicked() {
  generateTriangle(); 
}

// draw lines with arrowheads on the end
void arrow(int x1, int y1, int x2, int y2) {
  line(x1, y1, x2, y2);
  pushMatrix();
  translate(x2, y2);
  float a = atan2(x1-x2, y2-y1);
  rotate(a);
  line(0, 0, -(rad/2), -(rad/2));
  line(0, 0, (rad/2), -(rad/2));
  popMatrix();
} 

// convert two cartesian coordinates to a vector
PVector c2v(int Ax, int Ay, int Bx, int By) {
  return new PVector(Bx-Ax, By-Ay);
}

// given two coordinates and a length
// return a PVector defining the endpoint of a new line
// where if a line is drawn from the first given coordinate
// to the point defined by the new PVector
// the new line will be drawn along the same line
// defined by the given coordinates, but with new given length
PVector resizeVector (int x1, int y1, int x2, int y2, float newLength) {
  PVector initialVector = c2v(x1, y1, x2, y2);
  initialVector.normalize();
  float iVx = x1+(initialVector.x*newLength);
  float iVy = y1+(initialVector.y*newLength);
  return new PVector(iVx,iVy);
}

//********************************************************************
//* Barycentric Method for the Point in Triangle Test                *
//********************************************************************

boolean checkCollision(int t1x, int t1y, 
                       int t2x, int t2y, 
                       int t3x, int t3y, 
                       int px, int py) {
 
  // Compute vectors
  // t3 -> t1
  PVector v0 = c2v(t3x, t3y, t1x, t1y);
  // t2 -> t1
  PVector v1 = c2v(t2x, t2y, t1x, t1y);
  //  p -> t1
  PVector v2 = c2v(px, py, t1x, t1y);
  
  // Compute dot products
  float dot00 = v0.dot(v0);
  float dot01 = v0.dot(v1);
  float dot02 = v0.dot(v2);
  float dot11 = v1.dot(v1);
  float dot12 = v1.dot(v2);

  // Compute barycentric coordinates
  float invDenom = 1 / (dot00 * dot11 - dot01 * dot01);
  u = (dot11 * dot02 - dot01 * dot12) * invDenom;
  v = (dot00 * dot12 - dot01 * dot02) * invDenom;

  // Check if point is in triangle and return true/false
  return (u > 0) && (v > 0) && (u + v < 1);
  
}

//oooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo
//o BEGIN: Random triangle generation, can be ignored                o
//oooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo

// generate restricted coordinates within the draw space
// true returns an x coordinate
// false returns a y coordinate
int randCoord(boolean flag) {
  if(flag) {
    return int(random((rad*2)+(tSize*2),width-(rad*2)-(tSize*2))); 
  }
  else {
    return int(random((rad*2)+(tSize*4),height-(rad*2)-(tSize*2)));
  }
}

// generate random triangle with restricted area within the draw space
void generateTriangle() {
 
  // generate three pairs of coordinates
  t1x = randCoord(true);
  t1y = randCoord(false);
  
  t2x = randCoord(true);
  t2y = randCoord(false);
  
  t3x = randCoord(true);
  t3y = randCoord(false);
  
  // area of a triangle given 3 cartesian coordinates
  float area = 0.5*abs(((t1x-t3x)*(t2y-t1y))-((t1x-t2x)*(t3y-t1y)));
 
  // regenerate triangle if it is smaller than 1/12th the draw area
  if(area < (width*height)/12) generateTriangle();
  
}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
//x END: Random triangle generation, can be ignored                  x
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
