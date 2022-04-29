// Abbattista Valentina, Cifrignini Filippo, El Oukili Ossama

// Mediante la pressione dei tasti da 1 a 5 si seleziona il giunto corrispondente.
// Esso può quindi essere mosso in entrambi i versi usando le frecce destra e sinistra.
// Clickando con il mouse si può spostare la base del robot.
// Con le frecce SU e GIU si può modificare la vista verticale.
// Sullo schermo sono riportati in tempo reale il valore assunto dalle cinque variabili di giunto (θ1, d2, θ3, θ4 e θ5),
// le corrispondenti coordinate dell'estremità della pinza (x pinza, y pinza, z pinza) e il suo orientamento dato dalla matrice R05 (stampata a schermo R).



// parametro della funzione camera() che viene modificato con le 
// frecce SU e GIU e determina l'altezza della vista rispetto al robot
float eyeY = 0;

// Coordinate del centro del link 0 del robot che viene spostato col mouse
float xBase;
float yBase;

// Variabile per compattare le condizioni di fine corsa
int segno = 1;

// Permette di selezionare il giunto da muovere
int giunto = 0;

// dimensioni link 0:
float d0x = 70; // lungo x
float d0y = 50; // lungo y
float d0z = 70; // lungo z

// dimensioni link 1
float d1x = 100; // lungo x
float d1y = 20; // lungo y
float d1z = 20; // lungo z

// dimensioni link 2
float d2x = 100; // lungo x
float d2y = 20; // lungo y
float d2z = 20; // lungo z

// dimensioni link 3
float d3x = 20; // lungo x
float d3y = 20; // lungo y
float d3z = 150; // lungo z

// dimensioni link 4 
float d4x = 20; // lungo x
float d4y = 20; // lungo y
float d4z = 150; // lungo z

// dimensioni link 5
float d5x = 10; // lungo x
float d5y = 10; // lungo y
float d5z = 10; // lungo z


// parametri di giunto (theta1, d2, theta3, theta4, theta5)
float[] theta = {0,(d1x+d3x)/2,0,-PI/2,0};  

void setup() 
{
  size(1000, 600, P3D);
  stroke(255);
  strokeWeight(2);
  xBase = width/2;
  yBase = height/2;  
  textSize(25);
}


void draw() 
{
  
  background(0);
  lights();
// Permette di ruotare la vista:
  camera((width/2.0), height/2 - eyeY, (height/2.0) / tan(PI*60.0 / 360.0), width/2.0, height/2.0, 0, 0, 1, 0);  

  if (mousePressed)
  {
    xBase = mouseX;
    yBase = mouseY;
  }
  if (keyPressed)
  {
    // movimento camera
    if (keyCode == DOWN)
    {
      eyeY -= 5;
    }
    if (keyCode == UP)
    {
      eyeY += 5;
    }
    //scelta del giunto
    if (key == '1')
    {
      giunto = 0;
    }
    if (key == '2')
    {
      giunto = 1;
    }
    if (key == '3')
    {
      giunto = 2;
    }
    if (key == '4')
    {
      giunto = 3;
    }
    if (key == '5')
    {
      giunto = 4;
    }
   //scelta del verso di rotazione/traslazione 
   if (keyCode == LEFT)
    {
      segno = -1;
      muovi();
    }
    if (keyCode == RIGHT)
    {
      segno = 1;
      muovi();
    }
  }    
  
  
  pushMatrix(); // Memorizza il sistema di riferimento attuale

  fill(200,0,200); // Colore del robot
  
  // Link 0 (base)
  translate(xBase,yBase);
  box(d0x,d0y,d0z);
  
  
  //Sistema di riferimento della base
  stroke(255, 0, 0);
  line(0, d0y/2, 0, 200, d0y/2, 0); //asse y0
  stroke(0, 255, 0);
  line(0, d0y/2, 0, 0, d0y/2-200, 0); //asse z0
  stroke(0, 0, 255);
  line(0, d0y/2, 0, 0, d0y/2, +200); //asse x0
  stroke(255);
  fill(0,0,255);
  text("x0",5,0,160);
  fill(255,0,0);
  text("y0",160,-5,0);
  fill(0,255,0);
  text("z0",5,-160,0);
  fill(255,0,255);
  
  
  // Link 1 (si muove con theta1 = theta[0])
  rotateY(theta[0]);
  translate(0,-(d0y+d1y)/2,0);
  box(d1x,d1y,d1z);
   
  
  // Link 2 (si muove con d2 = theta[1])
  translate(d2y+theta[1]-(d1x+d3x)/2,0);
  box(d2x,d2y,d2z);  
  
  // Link 3 (si muove con theta3 = theta[2])
  rotateX(-theta[2]);
  translate((d2x-d3x)/2,0,(d3z-d2z)/2);  
  box(d3x,d3y,d3z);
  
  // Link 4 (si muove con theta4 = theta[3])
  translate(0,0,(d3z-d3y)/2);
  rotateX(-(theta[3]+PI/2));
  translate(0,0,(d4z-d3y)/2);
  box(d4x,d4y,d4z);
  
  // Link 5 (si muove con theta5 = theta[4])
  rotateZ(-theta[4]);
  translate(0,0,(d4z+d5z)/2);
  box(d5x,d5y,d5z);
  
  
  //Sistema di riferimento della pinza
  stroke(255, 0, 0);
  line(0, 0, d5z/2, -100, 0, d5z/2); //asse y5
  stroke(0, 0, 255);
  line(0, 0, d5z/2, 0, -100, d5z/2); //asse x5
  stroke(0, 255, 0);
  line(0, 0, d5z/2, 0, 0, d5z/2+100); //asse z5
  stroke(255);
  fill(0,0,255);
  text("x5",0,-50,0);
  fill(255,0,0);
  text("y5",-50,0,0);
  fill(0,255,0);
  text("z5",0,0,50);
  
    
  popMatrix();  // Ritorna al sistema di riferimento memorizzato
  
  //testo
  fill(255,0,0);
  text("giunto = ",10,20); 
  text(giunto+1,120,20);
  //parametri di giunto
  text("theta1 = ",10,70); 
  text((theta[0]*180/PI),120,70);
  text("d2 = ",10,120); 
  text(theta[1],75,120);
  text("theta3 = ",10,170); 
  text((theta[2]*180/PI),120,170);
  text("theta4 = ",10,220); 
  text((theta[3]*180/PI),120,220);
  text("theta5 = ",10,270); 
  text((theta[4]*180/PI),120,270);
  //coordinate pinza
  text("x pinza = ",10,320); 
  text((-(d4z+d5z-d4x/2)*(cos(theta[0])*cos(theta[2])*sin(theta[3])+cos(theta[0])*sin(theta[2])*cos(theta[3])))+(d4z-d4x)*cos(theta[0])*cos(theta[2])-sin(theta[0])*theta[1],130,320);
  text("y pinza = ",10,370); 
  text((-(d4z+d5z-d4x/2)*(sin(theta[0])*cos(theta[2])*sin(theta[3])+sin(theta[0])*sin(theta[2])*cos(theta[3])))+(d4z-d4x)*sin(theta[0])*cos(theta[2])+cos(theta[0])*theta[1],130,370);
  text("z pinza = ",10,420); 
  text(((-(d4z+d5z-d4x/2)*(-sin(theta[2])*sin(theta[3])+cos(theta[2])*cos(theta[3])))-(d4z-d4x)*sin(theta[2])+d0y+d1y/2),130,420);
  //orientamento
  text("R = ", 10, 520);
  text((cos(theta[0])*cos(theta[2])*cos(theta[3])-cos(theta[0])*sin(theta[2])*sin(theta[3]))*cos(theta[4])+sin(theta[0])*sin(theta[4]), 70, 470);
  text((-cos(theta[0])*cos(theta[2])*cos(theta[3])+cos(theta[0])*sin(theta[2])*sin(theta[3]))*sin(theta[4])+sin(theta[0])*cos(theta[4]), 180, 470);
  text(-cos(theta[0])*sin(theta[2])*cos(theta[3])-cos(theta[0])*cos(theta[2])*sin(theta[3]), 290, 470);
  text((sin(theta[0])*cos(theta[2])*cos(theta[3])-sin(theta[0])*sin(theta[2])*sin(theta[3]))*cos(theta[4])-cos(theta[0])*sin(theta[4]), 70, 520);
  text(-(sin(theta[0])*cos(theta[2])*cos(theta[3])-sin(theta[0])*sin(theta[2])*sin(theta[3]))*sin(theta[4])-cos(theta[0])*cos(theta[4]), 180, 520);
  text(-sin(theta[0])*cos(theta[2])*sin(theta[3])-sin(theta[0])*sin(theta[2])*cos(theta[3]), 290, 520);
  text((-sin(theta[2])*cos(theta[3])-cos(theta[2])*sin(theta[3]))*cos(theta[4]), 70, 570);
  text((sin(theta[2])*cos(theta[3])+cos(theta[2])*sin(theta[3]))*sin(theta[4]), 180, 570);
  text(sin(theta[2])*sin(theta[3])-cos(theta[2])*cos(theta[3]), 290, 570);
  //coordinata vista
  fill(0,255,0);
  text("coordinata y vista = ",500,30); 
  text(eyeY,750,30);
  
  

}




void muovi()
{
  if (giunto == 1) //traslazione giunto 2 con finecorsa
  {
    if (theta[1]>(d1x+d3x)/2 && theta[1]<(d1x-d3x)/2+d2x)
    {
      theta[giunto] += segno;
    }
    if (theta[1] == (d1x+d3x)/2 && segno == 1)
    {
      theta[giunto] += segno;
    }
    if (theta[1] == (d1x-d3x)/2+d2x && segno == -1)
    {
      theta[giunto] += segno;
    }
  }
  else
  {
    if(theta[giunto] > PI){
      theta[giunto] -= (2*PI);
    }
    if(theta[giunto] < -PI){
      theta[giunto] += (2*PI);
    }
    if (giunto == 0) //rotazione giunto 1
    {
      theta[giunto] += segno*.02;
    }
    if (giunto == 2)  //rotazione giunto 3
    {
      theta[giunto] += segno*.02;
    }
    if (giunto == 3) //rotazione giunto 4 con finecorsa
    {
      if ((theta[giunto]<82*PI/180 && segno<0) || (theta[giunto]>88*PI/180 && segno>0) || theta[giunto]<80*PI/180 || theta[giunto]>100*PI/180)
      {
        theta[giunto] += segno*.02;
      }
    }
      if (giunto == 4) //rotazione giunto 5
    {
      theta[giunto] += segno*.02;
    }
  }
}
