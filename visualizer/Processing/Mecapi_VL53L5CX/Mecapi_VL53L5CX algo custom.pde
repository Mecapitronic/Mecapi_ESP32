/*
  Visualizing VL53L5CX Output As A 3D Mesh
  By: Nick Poole
  SparkFun Electronics
  Date: November 3, 2021
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example is a companion to the Arduino Library Example 4 "MaxOutput"
  Which you need to be running on an attached device. You'll need to 
  change "COM13" below to the name of your serial port. 
  
  Once the sketch is running, it may take a few seconds for the sensor to respond,
  during which you'll see a red square in the window. If it takes more than a few
  seconds for this to change, check your serial port and run the sketch again. 
  
  Once data is incoming, you can rotate the mesh by dragging within the display window.
  You can zoom the mesh in and out using the scroll wheel on your mouse.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/18642
*/

import processing.serial.*;

// Serial Port Variables
Serial port; // Initialize Serial object
String comPort = "COM3"; // Serial com port number

String buff = ""; // Create a serial buffer
int[] depths = new int[64]; // Create a list to parse serial into 

// Mesh Generation Variables
int cols = 8; // Sensor is 8x8
int rows = 8;
//int scale = 100; // Scale value for drawing our mesh
float[][] terrain = new float[8][8]; // Create a table of distance values by pixel location

// Cursor Tracking Variables
float xPress = 0.0; // Store the cursor x position on mouse press 
float yPress = 0.0; // Store the cursor y position on mouse press

// Mesh Rotation Variables
float xRotOffset = 0; // Temporary x rotational offset (relevant during mouse drag)
float xRotPos = 0; // X rotational position
float yRotOffset = 0; // Temporary y rotational offset (relevant during mouse drag)
float yRotPos = 0; // Y rotational position
float zRotOffset = 0; // Temporary z rotational offset (relevant during mouse drag)
float zRotPos = 0; // Z rotational position
float transX = 0;
float transY = 0;
float transXpos = 0;
float transYpos = 0;
// Mesh Zoom Variables
float scaleOffset =0.7; // Scale factor from mouse wheel

  // sensor variables
float horFOVdeg = 45; // horizontal FOV in degrees
float verFOVdeg = 45; // horizontal FOV in degrees
int horRes = 8; // horizontal resolution
int verRes = 8; // vertical resolution
float horOFFSETdeg = 0; // horizontal offset angle in degrees
float verOFFSETdeg = -45; // vertical offset angle in degrees => 45+45/2
float zOFFSETmm = 315; // Z height fo sensor

float[] coefX = new float[8];
float[] coefY = new float[8];
float[] coefZ = new float[8];
float[] coefDistorsion = new float[8];
//point3D[] cartesianConv = new point3D[8];
//point3D[] cloud = new point3D[64]; // Create a table of point cloud in 3D
float[] pointX = new float[64];
float[] pointY = new float[64];
float[] pointZ = new float[64];

// Variables pour filtrage moyenne temporelle
int sizeMoyFilter = 10;
int[][] bufferMoyFilter = new int[64][sizeMoyFilter];
int[] indexMoyFilter = new int[64];
int[] sumMoyFilter = new int[64];

// variables d'objets physique
float distanceFrontObject = 1000;
float orientationObject = 0;
float pointLeft = 0;
float pointRight = 0;
int[] indexPOI = new int[64];

PVector[] points = new PVector[64];

//public class point3D {
//      public float x;
//      public float y;
//      public float z;
//      //public float radius;
//}

void setup(){
  String[] serials = Serial.list();
  
  size(1700,1700, P3D); // Make a 3D capable window
  //fullScreen();
  transX = width*0.2;
  transY = height*0.6;

  surface.setTitle("Mecapi VL53L5CX");
  surface.setResizable(true);
  surface.setLocation(10, 10);
  
  printArray(serials);
  if(serials.length>0)
  {
    for (int i = 0; i < serials.length; i = i+1)
    {
      if(serials[i].equals(comPort))
      {
        port = new Serial(this, comPort, 115200);
        port.clear();
      }
    }
  }
  
  // Fill our list with 0s to avoid a null pointer exception
  // in case the sensor data is not immediately available
  for(int idx = 0; idx < 64; idx++){
    depths[idx] = 300; // 30cm
  }
 
  // compute cartesian coefficients from sensor parameters
  float angle; // local variable
  // radians !
  float horAngRes = radians(horFOVdeg / (float)horRes);
  float verAngRes = radians(verFOVdeg / (float)verRes);
  float horAngZero = radians((horFOVdeg / 2) + horOFFSETdeg) - horAngRes/2;
  float verAngZero = radians((verFOVdeg / 2) + verOFFSETdeg) - verAngRes/2;

  // compute Vertical => Z and Y (both depend of vertical angle !)
  for (int i = 0; i < verRes; i++)
  {
      angle = verAngZero - (verAngRes * i);
      //angle = (verAngRes * i) - verAngZero;
      // (tentative de) correction de la distorsion de l'image, à calibrer sur sol plan
      coefDistorsion[i] = 1 + abs(sin(angle-radians(verOFFSETdeg)))*0.22;
      coefZ[i] = sin(angle) * coefDistorsion[i]; // coté opposée à l'angle => sinus
      coefY[i] = cos(angle) * coefDistorsion[i]; // Y en ordonnée, angle par rapport à axe Y => cosinus
  }
  // compute Horizontal => X
  for (int i = 0; i < horRes; i++)
  {
      //angle = (horAngRes * i) - horAngZero; // premier point à gauche vue de derrière
      angle = horAngZero - (horAngRes * i); // premier point à droite vue de derrière
      coefX[i] = sin(angle)  * coefDistorsion[i]; // X en abscisse, angle par rapport à axe Y => sinus
  }
}

float Y(float y) // in computer graphic interface, the Y axe is reversed !
{return -y;
}

// Fonction pour trier 3 valeurs et retourner la médiane
int mediane3(int a, int b, int c) 
{
    if (a > b) { int temp = a; a = b; b = temp; }  // a <= b
    if (b > c) { int temp = b; b = c; c = temp; }  // b <= c
    if (a > b) { int temp = a; a = b; b = temp; }  // a <= b (final)
    return b;  // Valeur médiane
}

void draw() // main loop
{
  // data acquisition here is faster than interrupt
  if(port != null)
  {
    buff = (port.readStringUntil(10)); // read the whole line
    if(buff!=null)
    {
      buff = buff.substring(0, buff.length()-2); // remove the Carriage Return
      if (buff != "") {
        //println(buff);
        depths = int(split(buff, ',')); // split at commas into our array
      }
    }
  }
  
  colorMode(HSB); // HSB color space makes it easy to map hue //<>//
  lights(); // Add ambient light to scene
  noStroke(); // Draw without stroke
  smooth(); // Draw with anti-aliasing
  background(0,0,120); // Fill background
  
//size(400,400,P3D);
//translate(232, 192, 0); 
//rotateY(0.5);
//noFill();

  
  textSize(32);
  float rotx = 0-(xRotOffset+xRotPos)+radians(60);
  float roty = 0+(yRotOffset+yRotPos);
  float rotz = 0+zRotOffset+zRotPos+radians(70);
  text("Rot X : " + degrees(rotx), 10, 20);
  text("Rot Y : " + degrees(roty), 10, 40);
  text("Rot Z : " + degrees(rotz), 10, 60);
  text("Zoom : " + scaleOffset, 10, 80);
  //text("distanceFrontObject : " + distanceFrontObject, 10, 100);
  
  //pushMatrix();
  // This stuff is all basically to scale and rotate the mesh
  // while keeping it in the center of the scene
  translate(transX,transY);
  rotateX(rotx);
  rotateY(roty);
  rotateZ(rotz);
  scale(scaleOffset);

  // Affichage origine et repère cartésien
  box(5);
  textSize(30);
  text("X+", 50, 0);
  text("Y+", 0, Y(50));
  text("Z+", 0, 0, 50);
  
  // Affichage FOV capteur
  stroke(0);
  line(0, 0, 320, -100, Y(100), 0);
  line(0, 0, 320, 100, Y(100), 0);
  line(0, 0, 320, -300, Y(800), 0);
  line(0, 0, 320, 300, Y(800), 0);
  // affichage projection au sol
  stroke(255);
  line(-100, Y(100), 0, 100, Y(100), 0);
  line(-100, Y(100), 0, -300, Y(800), 0);
  line(300, Y(800), 0, -300, Y(800), 0);
  line(300, Y(800), 0, 100, Y(100), 0);
  // affichage échelle au sol
  stroke(255);
  line(-10, Y(200), 0, 10, Y(200), 0); text("200", 0, Y(200));
  line(-10, Y(300), 0, 10, Y(300), 0); text("300", 0, Y(300));
  line(-10, Y(400), 0, 10, Y(400), 0); text("400", 0, Y(400));
  line(-10, Y(500), 0, 10, Y(500), 0); text("500", 0, Y(500));
  line(-10, Y(600), 0, 10, Y(600), 0); text("600", 0, Y(600));
  line(-10, Y(700), 0, 10, Y(700), 0); text("700", 0, Y(700));
  noStroke();

  // traitement du nuage de points
  if(depths.length >= 64)
  {
    // variables pour centre de gravité
    float center_x = 0;
    float center_y = 0;
    float sum_x = 0;
    float sum_y = 0;
    int num_points = 0;
    int k = 0;
    
    for(int i=0; i<64; i++) // for all points
    {
      // filtrage moyenne temporelle => stabilisation du bruit
      sumMoyFilter[i] -= bufferMoyFilter[i][indexMoyFilter[i]];  // Retirer l'ancienne valeur
      bufferMoyFilter[i][indexMoyFilter[i]] = depths[i];  // Ajouter la nouvelle mesure
      
        // filtrage médian => élimination des pics de bruit
        //depths[i] = mediane3(bufferMoyFilter[i][0], bufferMoyFilter[i][1], bufferMoyFilter[i][2]);
        
      sumMoyFilter[i] += depths[i];  
      indexMoyFilter[i] = (indexMoyFilter[i] + 1) % sizeMoyFilter;  // Avancer dans le buffer circulaire
      depths[i] = sumMoyFilter[i] / sizeMoyFilter;  // Calcul de la moyenne
        
      // conversion to cartesian coordinates
      int col = i % 8;  // plan horizontal => curseur
      int row = i / 8;  // plan vertical => numéro de ligne
      pointX[i] = depths[i]*coefX[col];
      pointY[i] = depths[i]*coefY[row]; // row because Y depend of vertical angle !
      pointZ[i] = depths[i]*coefZ[row] + zOFFSETmm;      
      //cloud[i].radius = 10;
      
      // calcul du centre de gravité (pour tout objet de 14cm de haut, +ou- 2cm)
      if ((pointZ[i] > 120) && (pointZ[i] < 160))
      {
        indexPOI[k] = i;
        k++;
        num_points++;
        sum_x += pointX[i];
        sum_y += pointY[i];
      }
      
      points[i] = new PVector(pointX[i], Y(pointY[i]), pointZ[i]); // points ransac
      
      // displaying version 1
      pushMatrix();
      translate(pointX[i], Y(pointY[i]), pointZ[i]); // position
      fill(map(pointZ[i],10,250,200,0),255,255); // couleur
      if (pointZ[i] < 20) box(depths[i]/100);
      else if ((pointZ[i] > 120) && (pointZ[i] < 160)) box(depths[i]/30);
      else sphere(depths[i]/50); // forme et taille
      textSize(15);
      fill(255);
      text((int)pointZ[i], 0, 0, 20);
      popMatrix();
    } // fin all points
    
    //  // Exécuter RANSAC pour trouver le meilleur segment
    //PVector[] rawSegment = ransacSegment();
    //PVector[] bestSegment = getSmoothedSegment(rawSegment);
  
  
    //// Afficher le segment trouvé
    //if (bestSegment != null) {
    //  stroke(0, 0, 255);
    //  strokeWeight(10);
    //  line(bestSegment[0].x, bestSegment[0].y, bestSegment[0].z, 
    //       bestSegment[1].x, bestSegment[1].y, bestSegment[1].z);
    //}
    //strokeWeight(1);
    
    //// affichage angle et distance
    //PVector capteur = new PVector(0, 0); // Position du capteur
    //if (bestSegment != null) {
    //  int angle = (int) computeAngle(bestSegment[0], bestSegment[1]) -180;
    //  while (angle <= -180) angle += 360;
    //  while (angle > 180) angle -= 360;
    //  int distance = (int) computeDistance(bestSegment[0], bestSegment[1], capteur);
    
    //  fill(255);
    //  textSize(20);
    //  text("Angle: " + angle + "°", 120, -100);
    //  text("Distance: " + distance + " mm", 120, -120);
    //}
    
ArrayList<PVector> pointList = new ArrayList<PVector>();
for (PVector p : points) {
    pointList.add(p);
}
  drawSegmentAtHeight(pointList, 140.0, 20.0); // Pour une planche à 140 mm de hauteur avec une tolérance de 20 mm

  
    //// recherche points extrêmes du bord
    //float minX = 1000;
    //float maxX = 0;
    //int indexminX = 0;
    //int indexmaxX = 0;
    //for (int i = 0; i < 64; i++) 
    //{
    //  if (indexPOI[i] != 0)
    //  {
    //    if (pointX[indexPOI[i]] < minX) 
    //    {
    //      minX = pointX[indexPOI[i]];
    //      indexminX = indexPOI[i];
    //    }
    //    if (pointX[indexPOI[i]] > maxX) 
    //    {
    //      maxX = pointX[indexPOI[i]];
    //      indexmaxX = indexPOI[i];
    //    }
    //  }
    //}
    //stroke(0, 0, 255);
    //strokeWeight(10);
    //line(pointX[indexminX], Y(pointY[indexminX]), pointZ[indexminX], pointX[indexmaxX], Y(pointY[indexmaxX]), pointZ[indexmaxX]);
    //strokeWeight(1);
    
    // fin calcul centre gravité et affichage
    center_x = sum_x / num_points;
    center_y = sum_y / num_points;
    pushMatrix();
    translate(center_x, Y(center_y), 0);
    box(10);
    fill(255);
    textSize(25);
    text((int)sqrt(center_x*center_x + Y(center_y)*Y(center_y)), 15, 0, 0);
    popMatrix();
    

    
    //for(int i=0; i<64; i++)
    //{
    //  int col = i % 8;  // plan horizontal => curseur
    //  int row = i / 8;  // plan vertical => numéro de ligne
    //  // displaying version 2
    //  beginShape(TRIANGLE_STRIP);
    //  fill(map(pointZ[i],10,250,200,0),255,255); // couleur
    //  vertex(pointX[col],Y(pointY[col]), depths[i]);
    //  vertex(pointX[row], Y(pointY[row]), depths[i+1]);
    //  endShape();
    //}
    fill(255);
  }
  
  // For each row, draw a triangle strip with the z-height of
  // each vertex corresponding to the distance reading at that 
  // location. While we're at it, set the fill color to a hue
  // corresponding to the data as well.
  //for(int y=0; y<rows-1; y++)
  //{
  //  beginShape(TRIANGLE_STRIP);
  //  for(int x=0; x<cols; x++)
  //  {
  //    fill(map(terrain[x][y],100,600,0,255),255,255);
  //    vertex(x*scale,y*scale, terrain[x][y]/1);
  //    vertex(x*scale, (y+1)*scale, terrain[x][y+1]/1);
  //  }
  //  endShape();
  //}

}
//************************************************************************************************************

float computeDistance(PVector p1, PVector p2, PVector capteur) {
  float num = abs((p2.y - p1.y) * capteur.x - (p2.x - p1.x) * capteur.y + p2.x * p1.y - p2.y * p1.x);
  float den = dist(p1.x, p1.y, p2.x, p2.y);
  return num / den;
}

float computeAngle(PVector p1, PVector p2) {
  return degrees(atan2(p2.y - p1.y, p2.x - p1.x)); // Convertit en degrés
}

// RANSAC pour trouver le segment le plus fiable
PVector[] ransacSegment() 
{
  int N_ITER = 100;  // Nombre d'itérations RANSAC
  float h = 140;  // Hauteur cible
  float distance_threshold = 10;  // Tolérance pour considérer un point comme inlier
  float y_threshold = 10; // Ajuste cette valeur selon la précision voulue
  int columns = 8; // Nombre de colonnes
float x_max = Float.MIN_VALUE; // Initialisation avec une valeur très faible
  
  ArrayList<PVector> filteredPoints = new ArrayList<PVector>();

  // 1️⃣ Filtrer les points proches de la hauteur h
  for (PVector p : points) {
    if (abs(p.z - h) < distance_threshold) {
      filteredPoints.add(p);
      // Calculer x_max (valeur maximale de x)
      if (p.x > x_max) {
        x_max = p.x;
      }
    }
  }

  if (filteredPoints.size() < columns) {
    println("Pas assez de points filtrés");
    return null; // Pas assez de points pour créer un segment
  }

  // 2️⃣ Trouver les points les plus proches en Y pour chaque colonne
  PVector[] closestPoints = new PVector[columns];

  // Initialiser les points les plus proches en y pour chaque colonne
  for (int i = 0; i < columns; i++) {
    closestPoints[i] = null; // Initialisation à null
  }

  // 3️⃣ Trouver le point le plus proche en y pour chaque colonne
  for (PVector p : filteredPoints) {
    // Mapper la coordonnée x en un index de colonne entre 0 et columns - 1
    int columnIndex = int(map(p.x, -x_max, x_max, 0, columns - 1)); // Mapper x à l'index de la colonne

    if (columnIndex >= 0 && columnIndex < columns) {
      // Si c'est le premier point de la colonne ou un point plus proche en y
      if (closestPoints[columnIndex] == null || abs(p.y) < abs(closestPoints[columnIndex].y)) {
        closestPoints[columnIndex] = p;
      }
    }
  }

  // 4️⃣ Construire le segment entre le premier et le dernier point des 8 points les plus proches
  PVector firstPoint = null;
  PVector lastPoint = null;

  for (PVector p : closestPoints) {
    if (firstPoint == null) {
      firstPoint = p;
    }
    lastPoint = p;
  }

  // Si on n'a pas trouvé de segment (pas assez de points)
  if (firstPoint == null || lastPoint == null) {
    println("Pas assez de points pour créer un segment");
    return null;
  }

  // 5️⃣ Retourner le segment formé par ces deux points
  return new PVector[]{firstPoint, lastPoint};
}



void drawSegmentAtHeight(ArrayList<PVector> points, float targetHeight, float tolerance) {
    ArrayList<PVector> leftCandidates = new ArrayList<>();
    ArrayList<PVector> rightCandidates = new ArrayList<>();
    PVector capteur = new PVector(0, 0);  // Position du capteur (origine)

    // Sélection des points dans la zone cible en hauteur
    for (PVector p : points) {
        if (abs(p.z - targetHeight) <= tolerance) {  // Vérifie la hauteur cible
            if (p.x < 0) {
                leftCandidates.add(p);
            } else {
                rightCandidates.add(p);
            }
        }
    }

    // Trier les listes par x (croissant pour left, décroissant pour right)
    leftCandidates.sort((p1, p2) -> Float.compare(p1.x, p2.x));
    rightCandidates.sort((p1, p2) -> Float.compare(p2.x, p1.x));

    // Calculer le nombre de points dans la zone cible
    int totalPoints = (int) points.stream().filter(p -> abs(p.z - targetHeight) <= tolerance).count();

    // Définir numPointExtreme comme 1/3 du total des points (arrondi vers le bas)
    int numPointExtreme = Math.max(4, totalPoints / 3);

    // Garder les numPointExtreme points les plus extrêmes de chaque côté
    if (leftCandidates.size() > numPointExtreme) leftCandidates.subList(numPointExtreme, leftCandidates.size()).clear();
    if (rightCandidates.size() > numPointExtreme) rightCandidates.subList(numPointExtreme, rightCandidates.size()).clear();

    // Trouver le plus proche du capteur parmi ces candidats
    PVector closestLeft = getClosestToCapteur(leftCandidates, capteur);
    PVector closestRight = getClosestToCapteur(rightCandidates, capteur);

    if (closestLeft != null && closestRight != null) {
        // Tracer le segment
        stroke(0, 0, 255);
        strokeWeight(5);
        line(closestLeft.x, closestLeft.y, targetHeight, closestRight.x, closestRight.y, targetHeight);
        strokeWeight(1);

        // 🔹 Affichage angle et distance
        int angle = (int) computeAngle(closestLeft, closestRight);
        while (angle <= -180) angle += 360;
        while (angle > 180) angle -= 360;
        int distance = (int) computeDistance(closestLeft, closestRight, capteur);

        fill(255);
        textSize(20);
        text("Angle: " + angle + "°", 120, -100);
        text("Distance: " + distance + " mm", 120, -120);
        text("Points: " + totalPoints, 120, -140);  // Ajout du nombre de points trouvés
    } else {
        println("Pas assez de points pour tracer un segment.");
    }
}




// Fonction pour trouver le point le plus proche du capteur
PVector getClosestToCapteur(ArrayList<PVector> candidates, PVector capteur) {
    PVector closest = null;
    float minDist = Float.MAX_VALUE;
    for (PVector p : candidates) {
        float dist = dist(capteur.x, capteur.y, p.x, p.y);
        if (dist < minDist) {
            minDist = dist;
            closest = p;
        }
    }
    return closest;
}


// Distance entre un point et un segment
float distancePointSegment(PVector P, PVector A, PVector B) {
  PVector AB = PVector.sub(B, A);
  PVector AP = PVector.sub(P, A);

  float t = AP.dot(AB) / AB.dot(AB);
  t = constrain(t, 0, 1);  // Clamping pour rester sur le segment

  PVector projection = PVector.add(A, PVector.mult(AB, t));
  return P.dist(projection);
}


int SMOOTHING_WINDOW = 5;  // Nombre de segments pour le lissage
ArrayList<PVector[]> previousSegments = new ArrayList<>();

PVector[] getSmoothedSegment(PVector[] newSegment) {
  if (newSegment == null) return null;

  // Ajouter le segment courant à l'historique
  previousSegments.add(newSegment);
  if (previousSegments.size() > SMOOTHING_WINDOW) {
    previousSegments.remove(0); // Garde seulement les derniers segments
  }

  // Moyenne des points extrêmes
  PVector avgP1 = new PVector(0, 0, 0);
  PVector avgP2 = new PVector(0, 0, 0);

  for (PVector[] seg : previousSegments) {
    avgP1.add(seg[0]);
    avgP2.add(seg[1]);
  }

  avgP1.div(previousSegments.size());
  avgP2.div(previousSegments.size());

  return new PVector[]{avgP1, avgP2};
}


//************************************************************************************************************
// When the mouse is pressed, remember where the cursor was
// so we can calculate how "far" it gets dragged
void mousePressed() {
  xPress = mouseX;
  yPress = mouseY;
  transXpos = transX;
  transYpos = transY;
}

// Between mouse drag events, draw will still be running, so we need to 
// update this offset at every event.
void mouseDragged() {
  if(mouseButton == LEFT)
  {
    xRotOffset = (mouseY-yPress)/200;
    zRotOffset = (mouseX-xPress)/200;
  }
  if(mouseButton == RIGHT)
  {
    xRotOffset = (mouseY-yPress)/200;
    yRotOffset = (mouseX-xPress)/200;
  }
  if(mouseButton == CENTER)
  {
    transX = transXpos + (mouseX-xPress);
    transY = transYpos + (mouseY-yPress);
  }
}

// To prevent the mesh position "snapping back" after releasing the mouse button
// We add the new temporary offset to the position and clear the temporary offset
// for the next mouse drag
void mouseReleased() {
  xRotPos += xRotOffset;
  xRotOffset = 0;
  yRotPos += yRotOffset;
  yRotOffset = 0;
  zRotPos += zRotOffset;
  zRotOffset = 0;
}

// Whenever the mousewheel is scrolled, we adjust the scale offset of the mesh
void mouseWheel(MouseEvent event) {
  float e = event.getCount();
  scaleOffset += e/10;
}
