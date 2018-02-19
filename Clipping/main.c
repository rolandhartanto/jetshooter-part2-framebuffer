/*
To test that the Linux framebuffer is set up correctly, and that the device permissions
are correct, use the program below which opens the frame buffer and draws a gradient-
filled red square:

retrieved from:
TesXting the Linux Framebuffer for Qtopia Core (qt4-x11-4.2.2)

http://cep.xor.aps.anl.gov/software/qt4-x11-4.2.2/qtopiacore-testingframebuffer.html
*/

#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <linux/fb.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <string.h>
#include <math.h>

#define HEIGHT 500
#define WIDTH 800
#define INIT_HEIGHT 100
#define INIT_WIDTH 100

struct fb_var_screeninfo vinfo;
struct fb_fix_screeninfo finfo;

typedef struct{
    int x, y;
}point;

typedef struct{
    point point1; //koord. titik awal
    point point2; //koord. titik akhir
}line;

typedef struct{
    line borders[1000];
}shape;

typedef struct{
    shape tail; // triangle
    shape left_wing; //triangle
    shape right_wing; //triangle
    point right_wing_center;
    point cockpit_center;
    int cockpit_radius;
    
    point body_center;
    int body_radius;
    
    point inner_center;
    int inner_radius;
}airplane;

airplane jet;
line hasil;//untuk mencatat hasil line clipping
char bg[1000][1000];
point stack[500000];
point stackTemp[500000];
int isVisited1[900][900];
int isVisited2[900][900];

char *fbp = 0;
int fbfd = 0;
long int screensize = 0;
long int location = 0;

//for cohen sutherland algo
const int INSIDE = 0;
const int LEFT = 1;
const int RIGHT = 2;
const int BOTTOM = 4;
const int TOP = 8;

//define const int x_max, y_max, x_min, y_min
const int x_min = 0;
const int y_min = HEIGHT;
const int x_max = WIDTH;
const int y_max = 0;

//untuk delay
const int iterationMax = 1e7;
int kodeBidang;

// menghitung region code of a point
int computeCode(point p);

//implement cohen sutherlan
void cohenSutherlandClipping(point p1, point p2);

void swap(int* a, int* b);
double DEG_to_RAD(double theta);

point setPoint(int x, int y);
point translatePoint(point p, int dx, int dy);
point rotatePoint(point p, double theta, point pivot);
point scalePoint(point p, double scale_factor, point pivot);

//line setLine(point x, point y);
//line translateLine(int dx, int dy);
//line rotateLine(line l, int theta, point pivot);
void loadFile();
void init();
void printPixel(int x, int y, int colorR, int colorG, int colorB);
void drawLine(point p1, point p2, int thickness, int colorR, int colorG, int colorB);
void drawCircle(point center, int radius, int colorR, int colorG, int colorB);
void clearScreen();
void delay();

//void drawPolygon(point center, int radius, int num_of_side, double theta);//STILL BUGGY(int&floating point problem), polygon with same side length, theta=0 start from center.y-radius

void getPixelColor(int x, int y, int *rColor, int *gColor, int *bColor);
int isBlack(int x, int y, int colorR, int colorG, int colorB);
void rasterize(int roffset, int coffset, int height, int width, int colorR, int colorG, int colorB);

void drawPropeller(int d1, int d2, point pivot, double theta, double scale_factor, int colorR, int colorG, int colorB);//theta = 0 start from pivot.x-d1/2
void drawAirplane(point offset, double scale_factor, double theta);

void floodFill(point source, int colorR, int colorG, int colorB);

int main() {
    init();   
    int i,j;
    
    loadFile();
    clearScreen();
    point p1 = setPoint(350,250);
    point p2 = setPoint(150,250);
    point pivot = setPoint(150+200,90+250);

    //printPixel(p1.x, p1.y ,255,0,0);
    
    //printPixel(pivot.x, pivot.y,255,0,0);
    //printf("%d\n",isBorder(pivot.x, pivot.y,255,0,0));
    //drawLine(pivot,p,1);

    point p1a,p2a;

    double k = 2.0;
    int enlarge = 1;
    int right = 1;
    int counter = 0;
    int x=200,y=250;
    double degree = 0;
    int counterGeser = 0;
    while(1){    
        for(i = 1; i < 360; i+=24){
            clearScreen();
            kodeBidang = 0;
            memset(isVisited1,0,sizeof isVisited1);
            
            // printf("mau gambar pesawat\n");
            if(counterGeser==10){

                for(j=1;j<=230;j+=10){
                    clearScreen();
                    // printf("j: %d\n",j);
                    point p = translatePoint(setPoint(x,y),-j,0);
                    memset(isVisited2,0,sizeof isVisited2);
                    drawAirplane(p,k,degree);

                    point pivot = translatePoint(setPoint(x+150,y+90),-j,0);
                    kodeBidang = 1;
                    memset(isVisited1,0,sizeof isVisited1);
                    
                    drawPropeller(120,30,pivot,i,k,150,0,20);
                    floodFill(pivot, 150,0,20);
                    // return 0;
                    // int offsetRow = pivot.y-120;
                    // int offsetCol = pivot.x-120;
                    // if(offsetRow<0){offsetRow = 0;}
                    // else if(offsetRow>=HEIGHT){offsetRow = HEIGHT-1;}
                    // if(offsetCol<0){offsetCol = 0;}
                    // else if(offsetCol>=WIDTH){offsetCol = WIDTH-1;}
                    // rasterize(offsetRow, offsetCol, 240,240,150,0,20);
                    delay(iterationMax);
                    kodeBidang = 0;
                }

                int jSekarang = j;
                // printf("jSekarang: %d\n",j);
                for(j=-jSekarang;j<=300;j+=10){
                    clearScreen();
                    // printf("j lagi: %d\n",j);
                    point p = translatePoint(setPoint(x,y),j,0);
                    memset(isVisited2,0,sizeof isVisited2);
                    
                    drawAirplane(p,k,degree);   
                    kodeBidang = 0;
                    point pivot = translatePoint(setPoint(x+150,y+90),j,0);

                    kodeBidang = 1;
                    memset(isVisited1,0,sizeof isVisited1);
                    
                    drawPropeller(120,30,pivot,i,k,150,0,20);
                    floodFill(pivot, 150,0,20);
                    // int offsetRow = pivot.y-120;
                    // int offsetCol = pivot.x-120;
                    // if(offsetRow<0){offsetRow = 0;}
                    // else if(offsetRow>=HEIGHT){offsetRow = HEIGHT-1;}
                    // if(offsetCol<0){offsetCol = 0;}
                    // else if(offsetCol>=WIDTH){offsetCol = WIDTH-1;}
                    // rasterize(offsetRow, offsetCol, 240,240,150,0,20);
                    delay(iterationMax);
                    kodeBidang = 0;
                }
                counterGeser = 0;
                return 0;
            }
            memset(isVisited2,0,sizeof isVisited2);
            drawAirplane(setPoint(x,y),k,degree);
            
            if((x < 230)&&(right)){
                x++;
                //degree -= 0.2;
            }else{
                right = 0;
                x--;
                //degree += 0.2;
                if(x == 170){
                    right = 1;
                }
            }
            
            //p1a = rotatePoint(p1, i, pivot);
            //p2a = rotatePoint(p2, i, pivot);
            //drawLine(p2a,p1a,1);
            // printf("mau gambar\n");
            kodeBidang = 1;
            drawPropeller(120,30,setPoint(x+150,y+90),i,k,150,0,20);
            floodFill(setPoint(x+150, y+90), 150,0,20);
            kodeBidang = 0;
            // printf("tergambar\n");
            
            if(counter >= 10){
	           k -= 0.12;
	        }
            // printf("masih\n");
            if(k >= 3.8){
                enlarge = 0;
                counter++;
            }
            // printf("masih2\n");
            if(k <= 1.0){
                enlarge = 1;
                counter = 0;
            }
            // printf("masih3\n");
            if(enlarge){
                k += 0.12;
            }
            // printf("counter: %d\n",counter);
            // printf("k: %.2lf\n",k);
            // printf("masih4\n");
            //printf("%lf\n",degree);
            // int offsetRow = pivot.y-120;
            // int offsetCol = pivot.x-120;
            // if(offsetRow<0){offsetRow = 0;}
            // else if(offsetRow>=HEIGHT){offsetRow = HEIGHT-1;}
            // if(offsetCol<0){offsetCol = 0;}
            // else if(offsetCol>=WIDTH){offsetCol = WIDTH-1;}
            // rasterize(offsetRow, offsetCol, 240,240,150,0,20);

            // printf("halo\n");
            counterGeser++;
            //printPixel(p1.x, p1.y, 0, 100, 255);
            delay(iterationMax);
        }
    }
    
    munmap(fbp, screensize);
    close(fbfd);
    return 0;
}
/***************** BASIC ******************/
void swap(int* a, int* b){
    int temp = *a;
    *a = *b;
    *b = temp;
}

double DEG_to_RAD(double theta){
    return (theta*M_PI/180.0);    
}

/***************** LOAD FILE ******************/
void loadFile(){
    FILE *fairplane;
    fairplane = fopen("../data/airplane.txt","r");
    if(fairplane == NULL) {
        printf("No data in font.txt\n");
        return 0;
    }
    else {
        int xa,xb,ya,yb;
        int num_of_line;
        printf("ok\n");
        for(int i = 1; i <= 6; i++){
            char dummy;
            fscanf(fairplane, "%d",&num_of_line);
            
            for(int j = 0; j<num_of_line; j++){
                //fscanf(fairplane, "%d %d %d %d",&xa,&ya,&xb,&yb);
                //printf("%d %d %d %d\n",xa,ya,xb,yb);
                if(i == 1){//tail
                    fscanf(fairplane, "%d %d %d %d",&xa,&ya,&xb,&yb);
                    jet.tail.borders[j].point1 = setPoint(xa,ya);
                    jet.tail.borders[j].point2 = setPoint(xb,yb);
                }else if(i == 2){//left wing
                    fscanf(fairplane, "%d %d %d %d",&xa,&ya,&xb,&yb);
                    jet.left_wing.borders[j].point1 = setPoint(xa,ya);
                    jet.left_wing.borders[j].point2 = setPoint(xb,yb);
                }else if(i == 3){//right wing
                    fscanf(fairplane, "%d %d %d %d",&xa,&ya,&xb,&yb);
                    jet.right_wing.borders[j].point1 = setPoint(xa,ya);
                    jet.right_wing.borders[j].point2 = setPoint(xb,yb);
                }else if(i == 4){//body
                    fscanf(fairplane, "%d %d %d",&xa,&ya,&xb);
                    jet.body_center = setPoint(xa,ya);
                    jet.body_radius = xb;
                }else if(i == 5){//inner
                    fscanf(fairplane, "%d %d %d",&xa,&ya,&xb);
                    jet.inner_center = setPoint(xa,ya);
                    jet.inner_radius = xb;
                }else if(i == 6){//cockpit
                    fscanf(fairplane, "%d %d %d",&xa,&ya,&xb);
                    jet.cockpit_center = setPoint(xa,ya);
                    jet.cockpit_radius = xb;
                }
                
                
            }
        }
    }
    jet.right_wing_center = setPoint(200,110);
    fclose(fairplane);
}

/***************** FRAMEBUFFER *****************/
void init(){
    fbfd = open("/dev/fb0", O_RDWR);
    if (fbfd == -1) {
        perror("Error: cannot open framebuffer device");
        exit(1);
    }
    printf("The framebuffer device was opened successfully.\n");

    // Get fixed screen information
    if (ioctl(fbfd, FBIOGET_FSCREENINFO, &finfo) == -1) {
        perror("Error reading fixed information");
        exit(2);
    }

    // Get variable screen information
    if (ioctl(fbfd, FBIOGET_VSCREENINFO, &vinfo) == -1) {
        perror("Error reading variable information");
        exit(3);
    }

    printf("%dx%d, %dbpp\n", vinfo.xres, vinfo.yres, vinfo.bits_per_pixel);

    // Figure out the size of the screen in bytes
    screensize = vinfo.xres * vinfo.yres * vinfo.bits_per_pixel / 8;

    // Map the device to memory
    fbp = (char *)mmap(0, screensize, PROT_READ | PROT_WRITE, MAP_SHARED, fbfd, 0);
    if ((int)fbp == -1) {
        perror("Error: failed to map framebuffer device to memory");
        exit(4);
    }
    printf("The framebuffer device was mapped to memory successfully.\n");

}

void clearScreen() {
    for (int h = 0; h < HEIGHT; h++){
        for (int w = 0; w < WIDTH; w++) {
	    printPixel(w,h,122,224,255);
        }
    }
}

void printPixel(int x, int y, int colorR, int colorG, int colorB){	//Print Pixel Color using RGB
    location = (x+vinfo.xoffset) * (vinfo.bits_per_pixel/8) +
                       (y+vinfo.yoffset) * finfo.line_length;

    if (vinfo.bits_per_pixel == 32) {
        *(fbp + location) = colorB;			//Blue Color
        *(fbp + location + 1) = colorG;		//Green Color
        *(fbp + location + 2) = colorR;		//Red Color
        *(fbp + location + 3) = 0;			//Transparancy
    }
}

void drawLine(point p1, point p2, int thickness, int colorR, int colorG, int colorB){//Bresenham
    int x1 = p1.x, y1 = p1.y, x2 = p2.x, y2 = p2.y;
    int steep = 0;
    if(abs(x1-x2) < abs(y1-y2)){
        swap(&x1, &y1);
        swap(&x2, &y2);
        steep = 1;
    }
    if(x1 > x2){
        swap(&x1,&x2);
        swap(&y1,&y2);
    }
    int dx = x2-x1;
    int dy = y2-y1;
    int derr = 2 * abs(dy);
    int err = 0;
    int y = y1;
    for(int x = x1; x <= x2; x++){
        if(steep){
            printPixel(y,x,colorR,colorG,colorB);
            if(kodeBidang==1){
                isVisited1[y][x] = 1;
            }else if(kodeBidang==2){
                isVisited2[y][x] = 1;
            }
        }else{
            printPixel(x,y,colorR,colorG,colorB);
            if(kodeBidang==1){
                isVisited1[x][y] = 1;
            }else if(kodeBidang==2){
                isVisited2[x][y] = 1;
            }
        }
        err+=derr;
        if(err > dx){
            y += (y2>y1)?1:-1;
            err -= 2 * dx;
        }
    }
}

void drawCircle(point center, int radius, int colorR, int colorG, int colorB){//Mid Point Algo
    int x = radius, y = 0;
    printPixel(x + center.x, y + center.y, colorR, colorG, colorB);
    if(radius > 0){
        printPixel(-x + center.x, y + center.y, colorR, colorG, colorB);
        printPixel(y + center.x, x + center.y, colorR, colorG, colorB);
        printPixel(y + center.x, -x + center.y, colorR, colorG, colorB);      
    }
    
    int P = 1 - radius;
    while(x > y){
        y++;
        if(P <= 0){
            P = P + 2*y + 1;
        }else{
            x--;
            P = P + 2*y - 2*x + 1;
        }

        if(x < y){ break; }
        printPixel(x + center.x, y + center.y, colorR, colorG, colorB);
        printPixel(-x + center.x, y + center.y, colorR, colorG, colorB);
        printPixel(x + center.x, -y + center.y, colorR, colorG, colorB);
        printPixel(-x + center.x, -y + center.y, colorR, colorG, colorB); 

        if(x != y){
            printPixel(y + center.x, x + center.y, colorR, colorG, colorB);
            printPixel(-y + center.x, x + center.y, colorR, colorG, colorB);
            printPixel(y + center.x, -x + center.y, colorR, colorG, colorB);
            printPixel(-y + center.x, -x + center.y, colorR, colorG, colorB); 

        }
    }
}

/*STILL BUGGY
void drawPolygon(point center, int radius, int num_of_side, double theta){
    int i;
    point p_start = rotatePoint(setPoint(center.x, center.y-radius), theta, center);
    point p_end = rotatePoint(p_start, 360.0/(double)num_of_side, center);
    drawLine(p_start, p_end, 1);
    //printf("%d %d", p_end.x, p_end.y);
    for(i = 2; i <= num_of_side; i++){
        drawLine(p_end,rotatePoint(p_end, 360.0/(double)num_of_side, center),1);
        p_end = rotatePoint(p_end, 360.0/(double)num_of_side, center);
        //printf("%d %d", p_end.x, p_end.y);
    }
}*/


/******************* COLORING *********************/

void getPixelColor(int x, int y, int *rColor, int *gColor, int *bColor) {
      location = (x+vinfo.xoffset) * (vinfo.bits_per_pixel/8) +
                       (y+vinfo.yoffset) * finfo.line_length;
            *bColor = *(fbp+location);
            *gColor = *(fbp+location+1);
            *rColor = *(fbp+location+2);
}

int isBorder(int x, int y, int colorR, int colorG, int colorB) {
    int rColorfb;
    int gColorfb;
    int bColorfb;
    getPixelColor(x,y,&rColorfb, &gColorfb, &bColorfb);
    
    rColorfb = (rColorfb < 0)? rColorfb + 256:rColorfb;
    gColorfb = (gColorfb < 0)? gColorfb + 256:gColorfb;
    bColorfb = (bColorfb < 0)? bColorfb + 256:bColorfb;
    
    return(rColorfb == colorR && gColorfb == colorG && bColorfb == colorB);
}

void rasterize(int roffset, int coffset, int height, int width, int colorR, int colorG, int colorB) {
    unsigned char onFlag = 0;
    unsigned char started = 0;
    for(int i = 0; i < height; i++) {
        int arr[width];
        int nPoint = 0;
        int y = roffset + i;

        for(int j = 0; j <= width; j++) {
            int x = coffset + j;
            if(isBorder(x,y,colorR,colorG,colorB)) {
                arr[nPoint] = j;
                while(isBorder(x,y,colorR,colorG,colorB)) {
                    j++;
                    x = coffset + j;
                }
                nPoint++;
            }
        }
        //printf("i: %d, nPoint: %d\n", i, nPoint);
        int median = -1;
        
        if(nPoint % 2 != 0) {
            median = nPoint / 2;
        }
       
        //printf("median: %d\n", median);
        //printf("Start Printing::\n");
        if(nPoint > 1 && i!=0 && i!=height) {
            for(int it = 0; it < nPoint-1; it+=2) {
                if(it == median) {
                    it++;
                }
                int startPoint = it;
                int endPoint = it+1;
                if(endPoint == median)
                    endPoint++;
                //printf("SP: %d %d EP %d %d\n", startPoint, arr[startPoint], endPoint, arr[endPoint]);
                if(endPoint < nPoint){
                    if(arr[endPoint] > arr[startPoint]){
                        for(int jt = arr[startPoint]; jt < arr[endPoint];jt++){
                            int x = coffset + jt;
                            printPixel(x,y,colorR,colorG,colorB);
                        }
                    }
                }
            }
        }
    }
}

/******************* POINT ********************/
point setPoint(int x, int y){
    point p;
    p.x = x;
    p.y = y;

    return p;
}

point translatePoint(point p, int dx, int dy){
    return setPoint(p.x + dx, p.y + dy);    
}

point rotatePoint(point p, double theta, point pivot){
    double rad = DEG_to_RAD(theta);
    int x = (int) ((double)(p.x-pivot.x) * cos(rad) - (double)(p.y-pivot.y) * sin(rad)) + pivot.x;
    int y = (int) ((double)(p.x-pivot.x) * sin(rad) + (double)(p.y-pivot.y) * cos(rad)) + pivot.y;
    
    return setPoint(x,y); 
}

point scalePoint(point p, double scale_factor, point pivot){
    int x = (int) ((double)(p.x-pivot.x) * scale_factor) + pivot.x;
    int y = (int) ((double)(p.y-pivot.y) * scale_factor) + pivot.y;
    
    return setPoint(x,y);
}

/************************* DRAW CUSTOM OBJECT *************************/
void drawPropeller(int d1, int d2, point pivot, double theta, double scale_factor, int colorR, int colorG, int colorB){
    point p1, p2, p3, p4, po1, pt1, po2, pt2, po3, pt3, po4, pt4;
    int i;
    point p[5];
    p[0] = setPoint(pivot.x-d1/2, pivot.y);
    p[1] = setPoint(pivot.x, pivot.y-d2/2);
    p[2] = setPoint(pivot.x+d1/2, pivot.y);
    p[3] = setPoint(pivot.x, pivot.y+d2/2);   

    for(i=0;i<4;i++){
        p[i] = rotatePoint(scalePoint(p[i], scale_factor, pivot), theta, pivot);
    }
    for(i=0;i<4;i++){
        if(i<3){
            cohenSutherlandClipping(p[i],p[i+1]);
            // printf("%d dengan %d\n",i,i+1);
        }else{
            cohenSutherlandClipping(p[3],p[0]);
            // printf("%d dengan %d\n",3,0);
        }
        
        po1 = hasil.point1; pt1 = hasil.point2;
        // printf("p1: %d %d p2: %d %d\n",po1.x,po1.y,pt1.x,pt1.y);
        drawLine(po1, pt1, 1, colorR, colorG, colorB);
    }
    
    // po1 = hasil.point1; pt1 = hasil.point2;
    // drawLine(po1, pt1, 1, colorR, colorG, colorB);
    
    

 //    point p1hasil, p2hasil;
 //    p1hasil = hasil.point1; p2hasil = hasil.point2;
 //    // if(p1hasil.x!=p1.x || p1hasil.y!=p1.y || p2hasil.y!=p1.x || p2hasil.y!=p1.x){}
 //    //printf("p1: %d %d p2: %d %d p1hasil:%d %d p2hasil: %d %d\n",p1.x,p1.y,p2.x,p2.y,p1hasil.x,p1hasil.y,p2hasil.x,p2hasil.y);
    

 //    cohenSutherlandClipping(p2,p3);
 //    p1hasil = hasil.point1; p2hasil = hasil.point2;
 //    //printf("p2: %d %d p3: %d %d p1hasil:%d %d p2hasil: %d %d\n",p2.x,p2.y,p3.x,p3.y,p1hasil.x,p1hasil.y,p2hasil.x,p2hasil.y);
 //    po2 = hasil.point1; pt2 = hasil.point2;
 //    drawLine(po2, pt2, 1, colorR, colorG, colorB);
 //    //bikin garis klo misal sama"x nya di x_max
    

 //    cohenSutherlandClipping(p3,p4);
 //    p1hasil = hasil.point1; p2hasil = hasil.point2;
 //    //printf("p3: %d %d p4: %d %d p1hasil:%d %d p2hasil: %d %d\n",p3.x,p3.y,p4.x,p4.y,p1hasil.x,p1hasil.y,p2hasil.x,p2hasil.y);
	// po3 = hasil.point1; pt3 = hasil.point2;
 //    drawLine(po3, pt3, 1, colorR, colorG, colorB);

 //    cohenSutherlandClipping(p4,p1);
 //    p1hasil = hasil.point1; p2hasil = hasil.point2;
 //    //printf("p4: %d %d p1: %d %d p1hasil:%d %d p2hasil: %d %d\n",p4.x,p4.y,p1.x,p1.y,p1hasil.x,p1hasil.y,p2hasil.x,p2hasil.y);
 //    po4 = hasil.point1; pt4 = hasil.point2;
 //    drawLine(po4, pt4, 1, colorR, colorG, colorB);

    // printf("p1: %d %d p2: %d %d p3: %d %d p4: %d %d\n",p1.x,p1.y,p2.x,p2.y,p3.x,p3.y,p4.x,p4.y);
}

void drawAirplane(point offset, double scale_factor, double theta){
    //left wing
    point p1,p2, pcolor;
    point pivot = setPoint(150+offset.x, 90+offset.y);
    point listPoint1[3], listPoint2[3];
    // printf("mau gambar left_wing\n");
    for(int i=0;i<3;i++){
        p1 = setPoint(jet.left_wing.borders[i].point1.x + offset.x, 
                      jet.left_wing.borders[i].point1.y + offset.y);
        p1 = scalePoint(p1, scale_factor, pivot);
        p1 = rotatePoint(p1, theta, pivot);

        p2 = setPoint(jet.left_wing.borders[i].point2.x + offset.x, 
                      jet.left_wing.borders[i].point2.y + offset.y);
        p2 = scalePoint(p2, scale_factor, pivot);
        p2 = rotatePoint(p2, theta, pivot);

        if(i == 0){
            pcolor = p1;
        }
        
        cohenSutherlandClipping(p1,p2);
        p1 = hasil.point1; p2 = hasil.point2;
        listPoint1[i] = p1; listPoint2[i] = p2;
        // printf("i: %d p1: %d %d p2: %d %d\n",i,p1.x, p1.y, p2.x, p2.y);
        // printf("listPoint1[%d]: %d %d listPoint2[%d]: %d %d\n",i,listPoint1[i].x,listPoint1[i].y,i,listPoint2[i].x,listPoint2[i].y);
        drawLine(p1, p2, 1, 107,91,0);    
    }
    // printf("mau gambar garis border\n");
    if(listPoint1[0].x==0 && listPoint2[2].x==0){
        p1 = listPoint1[0]; p2 = listPoint2[2];
        drawLine(p1, p2, 1, 107,91,0);
    }
    // printf("mau ngeraster\n");
    // int nilai = ;
    // if(nilai<0){nilai = 0;}
    int nilai1 = pcolor.y-50 * scale_factor;
    if(nilai1<0){nilai1 = 0;}
    else if(nilai1>=HEIGHT){nilai1 = HEIGHT-1;}
    int nilai2 = pcolor.x;
    if(nilai2<0){nilai2 = 0;}
    else if(nilai2>=WIDTH){nilai2 = WIDTH-1;}

    rasterize(nilai1, nilai2, 150 * scale_factor,150 * scale_factor,107,91,0);
    // printf("abis raster left wing\n");
    //right wing
    for(int i=0;i<3;i++){
        p1 = setPoint(jet.right_wing.borders[i].point1.x + offset.x, 
                      jet.right_wing.borders[i].point1.y + offset.y);
        p1 = scalePoint(p1, scale_factor, pivot);
        p1 = rotatePoint(p1, theta, pivot);

        p2 = setPoint(jet.right_wing.borders[i].point2.x + offset.x, 
                      jet.right_wing.borders[i].point2.y + offset.y);
        p2 = scalePoint(p2, scale_factor, pivot);
        p2 = rotatePoint(p2, theta, pivot);

        if(i == 0){
            pcolor = p1;
        }

        cohenSutherlandClipping(p1,p2);
        p1 = hasil.point1; p2 = hasil.point2;
        listPoint1[i] = p1; listPoint2[i] = p2;
        // printf("i: %d p1: %d %d p2: %d %d\n",i,p1.x, p1.y, p2.x, p2.y);
        // printf("listPoint1[%d]: %d %d listPoint2[%d]: %d %d\n",i,listPoint1[i].x,listPoint1[i].y,i,listPoint2[i].x,listPoint2[i].y);
        
        point pivotCenterRightWing;
        pivotCenterRightWing = setPoint(jet.right_wing_center.x + offset.x, jet.right_wing_center.y + offset.y);
        pivotCenterRightWing = scalePoint(pivotCenterRightWing, scale_factor, pivot);
        pivotCenterRightWing = rotatePoint(pivotCenterRightWing, theta, pivot);

        kodeBidang = 2;
        drawLine(p1, p2, 1, 107,91,0);
        if(pivotCenterRightWing.x<0){pivotCenterRightWing.x = 0;}
        else if(pivotCenterRightWing.x>=WIDTH){pivotCenterRightWing.x = WIDTH-1;}

        if(pivotCenterRightWing.y<0){pivotCenterRightWing.y = 0;}
        else if(pivotCenterRightWing.y>=HEIGHT){pivotCenterRightWing.y = HEIGHT-1;}
        // printf("pivot: %d %d\n",pivotCenterRightWing.x,pivotCenterRightWing.y);
        // floodFill(pivotCenterRightWing, 107, 91, 0);

        kodeBidang = 0;
    }
    // printf("mau gambar garis\n");
    if(listPoint2[0].x==WIDTH && listPoint1[1].x==WIDTH){
        p1 = listPoint2[0]; p2 = listPoint1[1];
        p1.x--; p2.x--; 
        drawLine(p1, p2, 1, 107,91,0);
    }
    // printf("mau raster\n");
    // nilai = ;
    // if(nilai<0){nilai = 0;}

    nilai1 = pcolor.y-50 * scale_factor;
    if(nilai1<0){nilai1 = 0;}
    else if(nilai1>=HEIGHT){nilai1 = HEIGHT-1;}
    nilai2 = pcolor.x-20;
    if(nilai2<0){nilai2 = 0;}
    else if(nilai2>=WIDTH){nilai2 = WIDTH-1;}
    rasterize(nilai1, nilai2, 150 * scale_factor,150 * scale_factor,107,91,0);

    // printf("abis raster right wing\n");
    // printf("abis raster\n");
    //tail
    for(int i=0;i<3;i++){
        p1 = setPoint(jet.tail.borders[i].point1.x + offset.x, 
                      jet.tail.borders[i].point1.y + offset.y);
        p1 = scalePoint(p1, scale_factor, pivot);
        p1 = rotatePoint(p1, theta, pivot);

        p2 = setPoint(jet.tail.borders[i].point2.x + offset.x, 
                      jet.tail.borders[i].point2.y + offset.y);
        p2 = scalePoint(p2, scale_factor, pivot);
        p2 = rotatePoint(p2, theta, pivot);

        if(i == 0){
            pcolor = p1;
        }
        
        cohenSutherlandClipping(p1,p2);
        p1 = hasil.point1; p2 = hasil.point2;
        drawLine(p1, p2, 1, 107,91,0);   
    }
    // printf("mau raster1\n");
    nilai1 = pcolor.y-20 * scale_factor;
    if(nilai1<0){nilai1 = 0;}
    else if(nilai1>=HEIGHT){nilai1 = HEIGHT-1;}
    nilai2 = pcolor.x-20 * scale_factor;
    if(nilai2<0){nilai2 = 0;}
    else if(nilai2>=WIDTH){nilai2 = WIDTH-1;}

    rasterize(nilai1, nilai2, 150 * scale_factor,150 * scale_factor,107,91,0);
    // printf("abis raster1\n");
    //cockpit
    p1 = setPoint(jet.cockpit_center.x + offset.x, jet.cockpit_center.y + offset.y);
    p1 = rotatePoint(scalePoint(p1,scale_factor,pivot),theta,pivot);
    drawCircle(p1, jet.cockpit_radius * scale_factor, 114,114,114);
    rasterize(p1.y - (jet.cockpit_radius+5) * scale_factor, 
              p1.x - (jet.cockpit_radius+5) * scale_factor, 
              2 * (jet.cockpit_radius+5) * scale_factor, 
              2 * (jet.cockpit_radius+5) * scale_factor, 
              114, 114, 114);
    // printf("abis raster2\n");
    // printf("abis raster cockpit\n");
    //body
    p1 = setPoint(jet.body_center.x + offset.x, jet.body_center.y + offset.y);
    drawCircle(p1, jet.body_radius * scale_factor,168,143,0);
    rasterize(p1.y - (jet.body_radius+5) * scale_factor, 
              p1.x - (jet.body_radius+5) * scale_factor, 
              2 * (jet.body_radius+5) * scale_factor, 
              2 * (jet.body_radius+5) * scale_factor, 
              168, 143, 0);
    // printf("abis raster3\n");
    // printf("abis raster body\n");
    //inner

    p1 = setPoint(jet.inner_center.x + offset.x, jet.inner_center.y + offset.y);
    drawCircle(p1, jet.inner_radius * scale_factor,107, 80,0);
    rasterize(p1.y - (jet.inner_radius+5) * scale_factor, 
              p1.x - (jet.inner_radius+5) * scale_factor, 
              2 * (jet.inner_radius+5) * scale_factor, 
              2 * (jet.inner_radius+5) * scale_factor, 
              107, 80, 0);
    // printf("abis raster4\n");
    // printf("abis raster inner\n");

    
}

int computeCode(point p){
    int x = p.x;
    int y = p.y;
    int code = INSIDE;
    if(x<x_min){code |= LEFT;}
    else if(x>x_max){code |= RIGHT;}

    if(y>y_min){code |= BOTTOM;}
    else if(y<y_max){code|=TOP;}

    return code;
}

void cohenSutherlandClipping(point p1, point p2){
    double x1 = p1.x, y1 = p1.y;
    double x2 = p2.x, y2 = p2.y;
    int code1 = computeCode(p1);
    int code2 = computeCode(p2);

    int isDrawn = 0;
    while(1){
        if((code1==0) && (code2==0)){ //2 endline di dlm rectangle yang akan digambar
            isDrawn = 1;
            break;
        }else if(code1 & code2){//hasil dan tidak 0, tidak digambar
            //bikin point -1
            point p1; p1.x = -1; p1.y = -1;
            point p2; p2.x = -1; p2.y = -1;
            hasil.point1 = p1;
            hasil.point2 = p2;
            break;
        }else{
            //harus diclip sebagian dari garis
            int code_out;
            double x,y;
            //ambil code yang di luar rectangle
            if(code1!=0){code_out = code1;}
            else{code_out = code2;}

            //find intersection point
            if(code_out&TOP){//point di atas rectangle
                x = x1+(x2-x1)*(y_max - y1) / (y2-y1);
                y = y_max;
            }else if(code_out & BOTTOM){//point di bawah rectangle
                x = x1 + (x2-x1)*(y_min-y1) / (y2-y1);
                y = y_min;
            }else if(code_out & RIGHT){
                y = y1 + (y2-y1) * (x_max-x1) / (x2-x1);
                x = x_max;
            }else if(code_out & LEFT){
                y = y1 + (y2-y1) * (x_min-x1) / (x2-x1);
                x = x_min;
            }

            //replace point outsite rectable dengan point yang di border
            if(code_out == code1){
                x1 = x;
                y1 = y;
                code1 = computeCode(p1);
            }else{
                x2 = x;
                y2 = y;
                code2 = computeCode(p2);
            }
            isDrawn = 1;
            break;
        }
    }
    if(isDrawn){
        point p1; p1.x = x1; p1.y = y1;
        point p2; p2.x = x2; p2.y = y2;
        hasil.point1 = p1;
        hasil.point2 = p2;
    }
}

void floodFill(point source, int colorR, int colorG, int colorB){
    int dx[] = {0,1,0,-1};
    int dy[] = {-1,0,1,0};

    // location = (x+vinfo.xoffset) * (vinfo.bits_per_pixel/8) +
    //                    (y+vinfo.yoffset) * finfo.line_length;

    // if (vinfo.bits_per_pixel == 32) {
    //     *(fbp + location) = colorB;         //Blue Color
    //     *(fbp + location + 1) = colorG;     //Green Color
    //     *(fbp + location + 2) = colorR;     //Red Color
    //     *(fbp + location + 3) = 0;          //Transparancy
    // }

    int i,j;
    int x = source.x, y = source.y;
    printPixel(x,y,colorR,colorG,colorB);
    //reset isVisited
    
    int ukuranStack = 1;
    stack[0] = source;
    if(kodeBidang==1){
        isVisited1[x][y] = 1;    
    }else if(kodeBidang==2){
        isVisited2[x][y] = 1;
    }
    // printf("pivot: %d %d\n",x,y);
    
    while(ukuranStack>0){
        int idxStackTemp = 0;
        int idxStack = 0;
        for(j=idxStack;j<ukuranStack;j++){
            x = stack[j].x; y = stack[j].y;

            for(i=0;i<4;i++){
                int xbaru = x+dx[i], ybaru = y+dy[i];
                // printf("x: %d y: %d xbaru: %d ybaru: %d\n",x,y,xbaru,ybaru);
                if(xbaru>=0 && xbaru<WIDTH && ybaru>=0 && ybaru<HEIGHT){
                    // location = (xbaru+vinfo.xoffset) * (vinfo.bits_per_pixel/8) +
                    //        (ybaru+vinfo.yoffset) * finfo.line_length;
                    // // printf("merah: %d hijau: %d biru: %d\n", *(fbp+location+2), *(fbp+location+1), *(fbp+location)); 
                    // // printf("intended: %d %d %d\n", colorR, colorG, colorB);
                    // if((*(fbp+location)==colorB) && (*(fbp+location+1)==colorG) && (*(fbp+location+2)==colorR)){
                    //     // printf("masuk kondisi sini\n");
                    //     continue;
                    // }
                    if(kodeBidang==1){
                        if(isVisited1[xbaru][ybaru]){continue;}    
                    }else if(kodeBidang==2){
                        if(isVisited2[xbaru][ybaru]){continue;}    
                    }
                    

                    
                    //masukin ke stack
                    point pBaru;
                    pBaru.x = xbaru; pBaru.y = ybaru;
                    stackTemp[idxStackTemp] = pBaru; idxStackTemp++;
                    // printf("idxStackTemp jadi: %d\n",idxStackTemp);
                    printPixel(xbaru,ybaru,colorR,colorG,colorB);
                    if(kodeBidang==1){
                        isVisited1[xbaru][ybaru] = 1;    
                    }else if(kodeBidang==2){
                        isVisited2[xbaru][ybaru] = 1;
                        // printf("yang dipush xbaru: %d ybaru: %d\n",xbaru,ybaru);
                    }

                    
                }
            }
        }
        //ukuranStack = 0;
        // printf("idxStackTemp: %d\n",idxStackTemp);
        for(i=0;i<idxStackTemp;i++){
            stack[i] = stackTemp[i];
        }
        ukuranStack = idxStackTemp;
        // printf("ukuranStack: %d\n",ukuranStack);
    }
    
}
void delay(int banyakIterasi){
    int i;
    for(i=0;i<banyakIterasi;i++){}
}