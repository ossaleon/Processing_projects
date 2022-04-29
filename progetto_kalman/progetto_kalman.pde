// Abbattista Valentina, Cinfrignini Filippo, El Oukili Ossama
// Le frecce nel codice indicano le parti che sono state aggiunte o modificate da noi

/*
Si disegna un robot mobile di tipo 'uniciclo'
e lo si controlla in velocità (senza limiti di saturazione)
verso il punto cliccato col mouse mediante la legge
di controllo proporzionale, tuttavia il calcolo del controllo
è fatto sulle variabili stimate anziché su quelle vere.
Sulla schermata viene anche disegnato (in giallo) il robot
nella posizione stimata (quello vero è disegnato in rosso).
In più, dei triangoli in bianco con bordo rosso rappresentano 
la posizione dei landmark presenti nell'ambiente. All'interno
di ciascun triangolo viene anche riportato un identificatore del
tipo Li per il landmark i-esimo.
Il robot possiede una regione di visibilità, la quale è rappresentata da un settore
circolare verde centrato nel robot. I landmark che cadono nella regione di visibilità
vengono evidenziati diventando bianchi (per evidenziarli solo quando si sta effettuando la misura, vedere riga 338).
Inoltre, il robot possiede una bussola che gli fornisce una misura del suo orientamento con un errore gaussiano con sigma pari a 10° (bussola = theta + ni).

Con le frecce è possibile modificare la frequenza delle
misurazioni della bussola e delle misurazioni della distanza dai landmark visibili o, più precisamente, il
tempo tStep che intercorre tra una misura e la successiva
(che comunque non può essere inferiore al ciclo di Processing,
per default dt = 1/60 secondo): con le frecce SU e GIU si 
incrementa e decrementa rispettivamente di 1 tale tempo mentre 
con le frecce DESTRA e SINISTRA lo si moltiplica e divide 
rispettivamente per 2 (per modificarlo più velocemente). Quando 
tale tempo è molto grande la ricostruzione diviene puramente 
odometrica.
Premendo i tasti 'r' e 'R' è possibile diminuire e, rispettivamente,
aumentare il range massimo rMax del settore circolare.
Analogamente, con i tasti 'b' e 'B', è possibile variare l'angolo di bearing
massimo betaMax che definisce l'apertura del settore circolare.

Sulla schermata vengono riportati: le coordinate
(x,y,theta) dell'uniciclo, le coordinate (xDes,yDes)
del punto da raggiungere e il tempo impiegato per la
missione. Inoltre si riportano le velocità angolari 
(omegaR, omegaL) delle due ruote, la velocità longitudinale 
v1 e quella angolare v2.
Si indicano anche le grandezze stimate (xHat,yHat,thetaHat)
con la loro matrice di covarianza P e il tempo tStep (modificabile 
da tastiera) che intercorre tra una lettura e la successiva.
Si riporta, infine, la misura della bussola e il numero di landmark visibili in un certo istante.
*/


// Dati per il settore circolare <--------
float rMax = 100; // raggio del settore
float betaMax = PI/6; // semiangolo del settore

// Dimensioni finestra
int sizeX = 1250;//1000;
int sizeY =  950;//550;

// Coordinate attuali uniciclo
float x = 0;
float y = 0;
float theta = 0;

// Bussola <--------
float bussola = theta; // variabile che contiene la misurazione della bussola <--------

// Coordinate desiderate uniciclo
float xDes = 0;
float yDes = 0;

// Caratteristiche fisiche uniciclo
float r = 8; // raggio ruote in pixel
float d = 25; // distanza tra le ruote in pixel
float w = 5; // spessore ruota in pixel
float R = 1.2*sqrt(pow(r,2)+pow(d/2+w/2,2)); // raggio robot

float dt = (float) 1/60; // tempo di campionamento

float e_p = 0; // errore posizionamento
float v1 = 0; // velocità lineare robot
float kv1 = 1; // costante legge proporzionale controllo v1
float v2 = 0; // velocità rotazionale robot
float kv2 = 2;  // costante legge proporzionale controllo v2
int nGiri = 0; // conta quanti giri su se stesso ha fatto il robot
float thetaDes; // orientamento desiderato (per raggiungere il target)
float t0,tempo; // tempo inizio e attuale missione
float tempoUltimaMisura; // tempo in cui si è effettuata l'ultima misura
long nStime = 0; // numero progressivo totale delle stime fatte

float omegaR = 0; // velocità angolare ruota destra
float omegaL = 0; // velocità angolare ruota sinistra
float uR, uL, uRe, uLe; // spostamenti ruote destra e sinistra veri e presunti (comandati)

// Variabili relative alla stima e al filtro di Kalman esteso
float sigmaX0 = 10; // deviazione standard dell'errore di stima del valore iniziale di x0
float sigmaY0 = 10; // deviazione standard dell'errore di stima del valore iniziale di y0
float sigmaTheta0 = 10*PI/180; // deviazione standard (in rad) dell'errore di stima del valore iniziale di theta0
float xHat = x + Gaussian(0,sigmaX0); // stima di x inizializzata al valore vero + una perturbazione gaussiana a media 0 e deviazione standard sigmaX0
float yHat = y + Gaussian(0,sigmaY0); // stima di y inizializzata al valore vero + una perturbazione gaussiana a media 0 e deviazione standard sigmaY0
float thetaHat = theta + Gaussian(0,sigmaTheta0); // stima di theta inizializzata al valore vero + una perturbazione gaussiana a media 0 e deviazione standard sigmaTheta0 rad
float xHatMeno,yHatMeno,thetaHatMeno; // stime a priori di x,y,theta
float KR = 0.01; // coefficiente varianza errore odometrico ruota destra
float KL = KR; // coefficiente varianza errore odometrico ruota sinistra
float stDevR,stDevL; // deviazione standard errore odometrico ruota destra e sinistra
//float [][] Landmark = {{0,sizeY/2-10}};  // Coordinate landmark (un solo landmark)
//float [][] Landmark = {{0,-sizeY/2},{0,sizeY/2}}; // Coordinate landmark (due landmark)
float [][] Landmark = {{-sizeX/2+10,-sizeY/2+10},{sizeX/2-10,-sizeY/2+10},{0,sizeY/2-10}};  // Coordinate landmark (3 landmark)
int nL = Landmark.length; // numero totale di landmark
int [] landmarkV = new int[nL]; // <-------- array di booleani che rappresenta quali landmark sono visibili
int nLv = 0; // <-------- numero landmark visibili
float[] misureLandmark = new float[nL]; // vettore con le misure vere di distanza dagli nL landmark
float[] misureAtteseLandmark = new float[nL]; // vettore con le misure attese di distanza dagli nL landmark
// Seguono le matrici utilizzate dal filtro di Kalman esteso (EKF)
float[][] F = {{1, 0, 0},{0, 1, 0},{0, 0, 1}}; // matrice giacobiana F=df/dx (alcuni elementi delle prime due righe vanno aggiustati durante l'esecuzione)
float[][] P = {{pow(sigmaX0,2), 0, 0},{0, pow(sigmaY0,2), 0},{0, 0, pow(sigmaTheta0,2)}}; // matrice di covarianza P inizializzata in base all'incertezza iniziale
float[][] Pmeno = new float[3][3]; // matrice di covarianza a priori P-
float[][] W = {{1, 0},{0,1},{1/d,-1/d}}; //  matrice giacobiana W = df/dw (gli elementi delle prime due righe vanno aggiustati durante l'esecuzione) 
float[][] Q = {{1, 0},{0,1}}; // matrice di covarianza del rumore di misura odometrico w (gli elementi sulla diagonale vanno aggiustati durante l'esecuzione)
float DeltaX,DeltaY,DeltaXY; // variabili di supporto
float sigmaLandmark = 15; // deviazione standard errore di misura di distanza dai landmark (in pixel)
float[][] correzione = new float[3][1]; // termine correttivo stima
float tStep = 0; // tempo (in ms) tra una misura e la successiva (impostabile da tastiera)

void setup() 
{
  size(1250,950);
  tempoUltimaMisura = 0; // Inizializzo a zero il tempo in cui è stata effettuata l'ultima misura
}

void draw() 
{
  
  background(0);

  pushMatrix();
  translate(sizeX/2,sizeY/2);

  if (keyPressed)
  {
    if (key == 'b' && betaMax > PI/12) // <-------- decrementa di 7.5° l'angolo betaMax se è maggiore di 15°
    {
      betaMax -= PI/24;
    }
    if (key == 'B' && betaMax < PI*5/12) // <-------- aumenta di 7.5° l'angolo betaMax se è minore di 75°
    {
      betaMax += PI/24;
    }
    if (key == 'r' && rMax > 50) // <-------- decrementa di 10 rMax se è maggiore di 50
    {
      rMax -= 10;
    }
    if (key == 'R' && rMax < 150) // <-------- aumenta di 10 rMax se è minore di 150
    {
      rMax += 10;
    }
    if (keyCode == UP) // aumento di 1 il tempo tra una misura e la successiva
    {
      tStep += 1;
    }
    if (keyCode == DOWN)  // decremento di 1 il tempo tra una misura e la successiva
    {
      tStep = max(0,tStep-1);
    }
    if (keyCode == RIGHT) // moltiplico per due il tempo tra una lettura e la successiva
    {
      tStep = tStep*2;
    }
    if (keyCode == LEFT) // divido per due il tempo tra una lettura e la successiva
    {
      tStep = tStep/2;
    }
  }
  
  if (mousePressed) // assegno target
  {
    xDes = mouseX - sizeX/2;
    yDes = sizeY/2 - mouseY;
    t0 = millis(); // inizio conteggio del tempo di missione
  }

// Calcolo errore e controllo basandomi sul valore stimato (xHat,yHat,thetaHat) delle variabili dell'uniciclo
  e_p = sqrt(pow(xDes-xHat,2)+pow(yDes-yHat,2));
  if (e_p > 1) // mi muovo solo se l'errore è maggiore di una certa quantità
  {
    tempo = (millis()-t0)/1000;  // tempo missione in secondi

    // assegno velocità secondo legge proporzionale (in termini delle quantità stimate!)
    v1 = -kv1*((xHat-xDes)*cos(thetaHat) + (yHat-yDes)*sin(thetaHat));

    // Calcolo l'angolo verso il target: scelgo il multiplo di 2PI 
    // più vicino all'orientamento corrente ***stimato*** del robot
    thetaDes = atan2(yDes-yHat,xDes-xHat) + nGiri*2*PI;
    if (abs(thetaDes+2*PI-thetaHat) < abs(thetaDes-thetaHat))
    {
      thetaDes = thetaDes+2*PI;
      nGiri += 1;
    }
    else
    {
      if (abs(thetaDes-2*PI-thetaHat) < abs(thetaDes-thetaHat))
      {
        thetaDes = thetaDes-2*PI;
        nGiri += -1;
      }
    }
    
   // assegno velocità angolare secondo legge proporzionale sempre in funzione delle quantità stimate   
    v2 = kv2*(thetaDes-thetaHat);
  }
  else // se penso di essere vicino al target mi fermo
  {
    v1 = 0;
    v2 = 0;
  }
  
  // Calcolo i movimenti da impartire alle ruote in base alle v1 e v2 trovate
  omegaR = (v1+v2*d/2)/r;
  omegaL = (v1-v2*d/2)/r;
  uRe = r*omegaR*dt; // spostamento comandato alla ruota destra (da considerarsi anche come informazione odometrica disponibile sul suo spostamento in (t,t+dt))
  uLe = r*omegaL*dt; // spostamento comandato alla ruota sinistra (da considerarsi anche come informazione odometrica disponibile sul suo spostamento in (t,t+dt))
  // Perturbo i due movimenti: gli spostamenti reali delle ruote non saranno mai esattamente uguali a quelli comandati 
  stDevR = sqrt(KR*abs(uRe));
  stDevL = sqrt(KL*abs(uLe));  
  uR = uRe + Gaussian(0,stDevR); // Spostamento vero ruota destra
  uL = uLe + Gaussian(0,stDevL); // Spostamento vero ruota sinistra
  
  // Dinamica effettiva dell'uniciclo
  x = x + ((uR+uL)/2)*cos(theta);
  y = y + ((uR+uL)/2)*sin(theta);
  theta = theta + (uR-uL)/d;
  bussola = theta + Gaussian(0, 10*PI/180); // <-------- definisco la bussola come l'angolo esatto theta con un errore gaussiano con sigma pari a 10°

  // STIMA FILTRO KALMAN ESTESO: PASSO di PREDIZIONE
  xHatMeno = xHat + ((uRe+uLe)/2)*cos(thetaHat);
  yHatMeno = yHat + ((uRe+uLe)/2)*sin(thetaHat);
  thetaHatMeno = thetaHat + (uRe-uLe)/d;

  //Aggiorno la giacobiana F (solo gli elementi variabili)
  F[0][2] = -(uRe+uLe)*sin(thetaHat)/2;
  F[1][2] = (uRe+uLe)*cos(thetaHat)/2;
  
  // Aggiorno W (solo gli elementi variabili)
  W[0][0] = .5*cos(thetaHat);
  W[0][1] = .5*cos(thetaHat);
  W[1][0] = .5*sin(thetaHat);
  W[1][1] = .5*sin(thetaHat);

  //Aggiorno Q (solo gli elementi variabili)
  Q[0][0] = KR*abs(uRe);
  Q[1][1] = KL*abs(uLe);
  
  // Calcolo matrice covarianza a priori
  Pmeno = mSum(mProd(mProd(F,P),trasposta(F)),mProd(mProd(W,Q),trasposta(W))); // Pmeno = F*P*F' + W*Q*W'


 // Verifico quali landmark sono visibili -------->
  nLv = 0;
  for (int indLandmark=0; indLandmark<nL; indLandmark++)
  {
    float angoloL = atan2(Landmark[indLandmark][1] - y, Landmark[indLandmark][0]-x); // angolo tra orizzontale e landmark
    if(sqrt(pow(x-Landmark[indLandmark][0],2)+pow(y-Landmark[indLandmark][1],2)) <= rMax && angoloL <= (theta-nGiri*2*PI + betaMax) &&  angoloL >= (theta - nGiri*2*PI - betaMax)) {
      landmarkV[indLandmark] = 1;
      nLv += 1;   
    }
    else {
      landmarkV[indLandmark] = 0;
    }
    
  }
 // <--------
 
 // Definizione delle matrici dipendenti dal numero di landmark visibili e che hanno una riga dedicata alla bussola  -------->
  float[][] H = new float[nLv+1][3]; // matrice giacobiana H = dh/dx
  float[][] K = new float[3][nLv+1]; // guadagno di Kalman
  float[][] Rs = idMat(nLv+1,pow(sigmaLandmark,2)); // matrice di covarianza errore misura dai landmark
  Rs[nLv][nLv] = pow(10*PI/180,2);  // <-------- l'ultima varianza è quella della bussola
  float[][] innovazione = new float[nLv+1][1]; // innovazione EKF
 // <--------
 
 // STIMA FILTRO DI KALMAN ESTESO: PASSO di CORREZIONE
  if (millis()-tempoUltimaMisura >= tStep) // attuo la correzione solo se ho le misure (che arrivano ogni tStep ms)
  {
    tempoUltimaMisura = millis(); // memorizzo il tempo in cui ho fatto l'ultima misura
    nStime++; // incremento il contatore delle stime fatte
    //la bussola è sempre accessibile, quindi in primis calcolo l'ultima riga di H e di innovazione (riga riferita alla bussola) -------->
    H[nLv][0] = 0;  // per la bussola, z = theta + ni, allora dz/dx = 0;
    H[nLv][1] = 0;  // dz/dy = 0;
    H[nLv][2] = 1;  // dz/dtetha = 1.
    innovazione[nLv][0] = bussola - thetaHatMeno;
    // <--------
    
    int ind = 0; // <-------- indice di supporto compreso tra 0 e nLv per scrivere sulle matrici definite a riga 270
    // per ogni landmark visibile calcolo misura vera, attesa, la riga della
    // matrice giacobiana H e l'innovazione corrispondente
    for (int indLandmark=0; indLandmark<nL; indLandmark++) 
    {
      if(landmarkV[indLandmark] == 1){ // <-------- calcolo solo per i landmark visibili
          misureLandmark[ind] = sqrt(pow(x-Landmark[ind][0],2) + pow(y-Landmark[ind][1],2))+Gaussian(0,sigmaLandmark);
          misureAtteseLandmark[ind] = sqrt(pow(xHatMeno-Landmark[ind][0],2) + pow(yHatMeno-Landmark[ind][1],2));
          DeltaX = xHatMeno-Landmark[ind][0];
          DeltaY = yHatMeno-Landmark[ind][1];
          DeltaXY = sqrt(pow(DeltaX,2)+pow(DeltaY,2));
          H[ind][0] = DeltaX/DeltaXY;
          H[ind][1] = DeltaY/DeltaXY;  
          H[ind][2] = 0;
          innovazione[ind][0] = misureLandmark[ind]-misureAtteseLandmark[ind];
          ind += 1; // <-------- incremento l'indice dopo ogni iterazione su un landmark visibile
      }
    }
    
    // Calcolo guadagno Kalman e aggiorno covarianza
    K = mProd(mProd(Pmeno,trasposta(H)),invMat(mSum(mProd(mProd(H,Pmeno),trasposta(H)),Rs)));
    P = mProd(mSum(idMat(3,1),mProd(idMat(3,-1),mProd(K,H))),Pmeno);

    // Correggo la stima    
    correzione = mProd(K,innovazione);
    xHat = xHatMeno + correzione[0][0];
    yHat = yHatMeno + correzione[1][0];
    thetaHat = thetaHatMeno + correzione[2][0];    
  }
  else  // se non ho misure non correggo nulla
  {
    xHat = xHatMeno;
    yHat = yHatMeno;
    thetaHat = thetaHatMeno;
    P = Pmeno;
  }
// FINE EKF

// Disegno il robot vero e quello stimato
  robot(x,y,theta,1); // l'argomento 1 fa un robot rosso (robot reale)
  robot(xHat,yHat,thetaHat,0); // l'argomento 0 un robot giallo (robot nella posa stimata)

// Disegno i landmark con dei triangoli bianchi col contorno rosso e l'identificativo del landmark all'interno
  stroke(255,0,0);
  strokeWeight(2);  
  for (int indLandmark=0; indLandmark<nL;indLandmark++)
  {
    fill(100,100,100);
    if(landmarkV[indLandmark] == 1 /*&& millis()-tempoUltimaMisura >= tStep*/){ // il landmark si illumina se è visibile, anche se non stiamo effettuando la misura. Togliendo il commento, si illuminerà solo se stiamo effettuando la misura
      fill(255,255,255);
    }
    triangle(Landmark[indLandmark][0]-15,-Landmark[indLandmark][1]+15,Landmark[indLandmark][0]+15,-Landmark[indLandmark][1]+15,Landmark[indLandmark][0],-Landmark[indLandmark][1]-15);
    textSize(10);
    fill(0,0,0);
    text("L",Landmark[indLandmark][0]-5,-Landmark[indLandmark][1]+8);
    text(indLandmark+1,Landmark[indLandmark][0]+1,-Landmark[indLandmark][1]+8);
  }
  stroke(0);
  strokeWeight(1);
  
  popMatrix();

  textSize(20);
  fill(0,0,255);
  text("v1 (pixel/s) = ",10,20); 
  text(v1,200,20);
  text("v2 (gradi/s) = ",10,50); 
  text(v2*180/PI,200,50);
  
  fill(255,0,0);  
  text("x = ",10,160); 
  text(x,80,160);
  text("y = ",10,190); 
  text(y,80,190);
  text("theta = ",10,220); 
  text(theta*180/PI,100,220);  

  fill(255,255,255);
  text("tempo = ",10,110); 
  text(tempo,120,110);  

  fill(0,0,255);
  text("omegaR (gradi/s) = ",650,20); 
  text(omegaL*180/PI,900,20);
  text("omegaL (gradi/s) = ",650,50); 
  text(omegaL*180/PI,900,50);
  
  fill(255,0,0);
  text("xDes = ",700,100); 
  text(xDes,800,100);
  text("yDes = ",700,130); 
  text(yDes,800,130);
  text("bussola = ",700,160); // <--------
  text(bussola*180/PI,810,160);
  text("landmark visibili = ", 700, 190); // <--------
  text(nLv, 890, 190);


  fill(255,255,0);  
  text("xHat = ",10,280); 
  text(xHat,120,280);
  text("yHat = ",10,310); 
  text(yHat,120,310);
  text("thetaHat = ",10,340); 
  text(thetaHat*180/PI,160,340);  

  fill(255,255,255);
  text("nStime = ",10,390); 
  text(nStime,120,390);  

  fill(255,255,255);
  text("tStep (ms) = ",10,420); 
  text(tStep,150,420);  

  fill(255,255,0);  
  text("P = ",10,460); 
  text(P[0][0],10,490); text(P[0][1],100,490); text(P[0][2],190,490);
  text(P[1][0],10,520); text(P[1][1],100,520); text(P[1][2],190,520); 
  text(P[2][0],10,550); text(P[2][1],100,550); text(P[2][2],190,550);  
}

void robot(float x, float y, float theta, int colore)
{
// funzione che disegna uniciclo in (x,y) con orientamento theta      
  pushMatrix();
  translate(x,-y);
  rotate(-theta);
  if (colore==1)
  {
    fill(0,255,0);
    arc(0, 0, 2*rMax, 2*rMax, -betaMax, betaMax); // <-------- disegno in verde il settore con raggio rMax e angolo betaMax
    fill(255,0,0);
    
  }
  else
  {
    fill(255,255,0);
  }
  ellipse(0,0,2*R,2*R); // il robot
  fill(0,0,255);  
  rect(-r,-d/2-w/2,2*r,w);
  rect(-r,d/2-w/2,2*r,w);
  fill(255);
  ellipse(.8*R,0,.2*R,.2*R);
  ellipse(-.8*R,0,.2*R,.2*R);  
  fill(0,255,0);
  triangle(-.1*R,.3*R,-.1*R,-.3*R,.5*R,0);
  popMatrix();
}


/******************************************************
/******************************************************
  DA QUI IN POI CI SONO FUNZIONI DI SERVIZIO: 
  1) CALCOLO ERRORE GAUSSIANO
  2) ALGEBRA MATRICIALE
/******************************************************
/******************************************************/

float Gaussian(float media, float stDev) // Restituisce variabile N(media,stDev^2) approssimata sfruttando il teorema del limite centrale
{
  float somma = 0;
  for (int k=0; k<27; k+=1) // 27 in modo che sqrt(3/27)=1/3
  {
    somma = somma + random(-stDev/3,stDev/3);
  }
  return media + somma;
}

float[][] mProd(float[][] A,float[][] B) // Calcola prodotto di due matrici A e B (si assume, senza controllarlo, che numero colonne A = numero righe B!)
{
  int nA = A.length;
  int nAB = A[0].length;
  int nB = B[0].length;
  
  float[][] C = new float[nA][nB]; 

  for (int i=0; i < nA; i++) 
  {
    for (int j=0; j < nB; j++) 
    {  
      for (int k=0; k < nAB; k++) 
      {
        C[i][j] += A[i][k] * B[k][j];
      }
    }
  }
  return C;
}

float[][] mSum(float[][] A,float[][] B) // Calcola la somma di due matrici A e B (si assume, senza controllarlo, che A e B abbiano stesse dimensioni!)
{
  int nA = A.length;
  int nB = A[0].length;
  
  float[][] C = new float[nA][nB]; 

  for (int i=0; i < nA; i++) 
  {
    for (int j=0; j < nB; j++) 
    {  
      C[i][j] = A[i][j] + B[i][j];
    }
  }
  return C;
}

float[][] trasposta(float[][] A) // Calcola la trasposta di una matrice A
{
  int nR = A.length;
  int nC = A[0].length; 
  
  float[][] C = new float[nC][nR]; 

  for (int i=0; i < nC; i++) 
  {
    for (int j=0; j < nR; j++) 
    {  
      C[i][j] = A[j][i];
    }
  }
  return C;
}


float[][] minore(float[][] A, int i, int j) // Determina il minore (i,j) di una matrice A (si assume, senza controllarlo, che A sia quadrata!)
{
  int nA = A.length;
  float[][] C = new float[nA-1][nA-1];
  
  for (int iM = 0; iM < i; iM++)
  {
    for (int jM = 0; jM < j; jM++)
    {
      C[iM][jM] = A[iM][jM];
    } 
    for (int jM = j; jM < nA-1; jM++)
    {
      C[iM][jM] = A[iM][jM+1];
    } 
  }
  for (int iM = i; iM < nA-1; iM++)
  {
    for (int jM = 0; jM < j; jM++)
    {
      C[iM][jM] = A[iM+1][jM];
    } 
    for (int jM = j; jM < nA-1; jM++)
    {
      C[iM][jM] = A[iM+1][jM+1];
    } 
  }
  return C;
}


float det(float[][] A) // Calcola il determinante di A (si assume, senza controllarlo, che A sia quadrata!)
{
  int nA = A.length;
  float determinante = 0;
  
  if (nA == 1)
  {
    determinante = A[0][0];
  }
  else
  {
    for (int j=0; j < nA; j++) 
    {
      determinante = determinante + A[0][j]*pow(-1,j)*det(minore(A,0,j));
    }
  }
  return determinante;
}


float[][] invMat(float[][] A) // Calcola l'inversa di una matrice A (si assume, senza controllarlo, che A sia quadrata!)
{
  int nA = A.length;
  float[][] C = new float[nA][nA];
  float detA = det(A);

/*
  if (abs(detA)<0.001) // Per evitare casi singolari con determinanti vicini a 0
  {
    if (detA>0)
    {
      detA = 0.001;
    }
    else
    {
      detA = -0.001;
    }
  }
*/  
  
  if (nA == 1)
  {
    C[0][0] = 1/detA;
  }
  else
  {
    for (int i=0; i < nA; i++) 
    {
      for (int j=0; j < nA; j++) 
      {
        C[j][i] = pow(-1,i+j)*det(minore(A,i,j))/detA;
      }
    }
  }
  return C;
}

float[][] idMat(int nA, float sigma) // Assegna una matrice identità di ordine nA moltiplicata per una costante sigma
{
  float[][] I = new float[nA][nA]; 

  for (int i=0; i < nA; i++) 
  {
    for (int j=0; j < nA; j++) 
    {  
      I[i][j] = 0;
    }
    I[i][i] = sigma;
  }
  return I;
}
