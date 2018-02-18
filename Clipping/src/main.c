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
    
    point cockpit_center;
    int cockpit_radius;
    
    point body_center;
    int body_radius;
    
    point inner_center;
    int inner_radius;
}airplane;

airplane jet;
char bg[1000][1000];

char *fbp = 0;
int fbfd = 0;
long int screensize = 0;
long int location = 0;

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

//void drawPolygon(point center, int radius, int num_of_side, double theta);//STILL BUGGY(int&floating point problem), polygon with same side length, theta=0 start from center.y-radius

void getPixelColor(int x, int y, int *rColor, int *gColor, int *bColor);
int isBlack(int x, int y, int colorR, int colorG, int colorB);
void rasterize(int roffset, int coffset, int height, int width, int colorR, int colorG, int colorB);

void drawPropeller(int d1, int d2, point pivot, double theta, double scale_factor, int colorR, int colorG, int colorB);//theta = 0 start from pivot.x-d1/2
void drawAirplane(point offset, double scale_factor, double theta);

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
    while(1){    
        for(i = 1; i < 360; i+=24){
            clearScreen();
            drawAirplane(setPoint(x,y),k,degree);
            
            if((x < 230)&&(right)){
                x++;
                degree -= 0.2;
            }else{
                right = 0;
                x--;
                degree += 0.2;
                if(x == 170){
                    right = 1;
                }
            }
            
            //p1a = rotatePoint(p1, i, pivot);
            //p2a = rotatePoint(p2, i, pivot);
            //drawLine(p2a,p1a,1);
            drawPropeller(120,30,setPoint(x+150,y+90),i,k,150,0,20);
            if(counter >= 50){
	        k -= 0.02;
	    }
            if(k >= 4.3){
                enlarge = 0;
                counter++;
            }
            
            if(k <= 2.7){
                enlarge = 1;
                counter = 0;
            }

            if(enlarge){
                k += 0.02;
            }

            //printf("%lf\n",degree);
            //rasterize(pivot.y-120, pivot.x-120, 240,240,150,0,20);
            
            //printPixel(p1.x, p1.y, 0, 100, 255);
            for(j = 0; j < 10000000; j++){}
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
            printPixel((y > WIDTH) ?WIDTH-1 :((y < 0) ?0 :y),(x > HEIGHT) ?HEIGHT-1 :((x < 0) ?0 :x),colorR,colorG,colorB);
        }else{
            printPixel((x > WIDTH) ?WIDTH-1 :((x < 0) ?0 :x),(y > HEIGHT) ?HEIGHT-1 :((y < 0) ?0 :y),colorR,colorG,colorB);
        }

        err+=derr;
        if(err > dx){
			if (y >= 0 && y < HEIGHT){
				y += (y2>y1)?1:-1;
			}
			err -= 2 * dx;
        }
    }
}

void drawCircle(point center, int radius, int colorR, int colorG, int colorB){//Mid Point Algo
    int x = radius, y = 0;
    printPixel((x + center.x > WIDTH-1) ?WIDTH-1 :((x + center.x < 0) ?0 :(x + center.x)), (y + center.y > HEIGHT-1) ?HEIGHT-1 :((y + center.y < 0) ?0 :(y + center.y)), colorR, colorG, colorB);
    if(radius > 0){
        printPixel((-x + center.x > WIDTH-1) ?WIDTH-1 :((-x + center.x < 0) ?0 :(-x + center.x)), (y + center.y > HEIGHT-1) ?HEIGHT-1 :((y + center.y < 0) ?0 :(y + center.y)), colorR, colorG, colorB);
        printPixel((y + center.x > WIDTH-1) ?WIDTH-1 :((y + center.x < 0) ?0 :(y + center.x)), (x + center.y > HEIGHT-1) ?HEIGHT-1 :((x + center.y < 0) ?0 :(x + center.y)), colorR, colorG, colorB);
        printPixel((y + center.x > WIDTH-1) ?WIDTH-1 :((y + center.x < 0) ?0 :(y + center.x)), (-x + center.y > HEIGHT-1) ?HEIGHT-1 :((-x + center.y < 0) ?0 :(-x + center.y)), colorR, colorG, colorB);      
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
        printPixel((x + center.x > WIDTH-1) ?WIDTH-1 :((x + center.x < 0) ?0 :(x + center.x)), (y + center.y > HEIGHT-1) ?HEIGHT-1 :((y + center.y < 0) ?0 :(y + center.y)), colorR, colorG, colorB);
        printPixel((-x + center.x > WIDTH-1) ?WIDTH-1 :((-x + center.x < 0) ?0 :(-x + center.x)), (y + center.y > HEIGHT-1) ?HEIGHT-1 :((y + center.y < 0) ?0 :(y + center.y)), colorR, colorG, colorB);
        printPixel((x + center.x > WIDTH-1) ?WIDTH-1 :((x + center.x < 0) ?0 :(x + center.x)), (-y + center.y > HEIGHT-1) ?HEIGHT-1 :((-y + center.y < 0) ?0 :(-y + center.y)), colorR, colorG, colorB);
        printPixel((-x + center.x > WIDTH-1) ?WIDTH-1 :((-x + center.x < 0) ?0 :(-x + center.x)), (-y + center.y > HEIGHT-1) ?HEIGHT-1 :((-y + center.y < 0) ?0 :(-y + center.y)), colorR, colorG, colorB);

        if(x != y){
            printPixel((y + center.x > WIDTH-1) ?WIDTH-1 :((y + center.x < 0) ?0 :(y + center.x)), (x + center.y > HEIGHT-1) ?HEIGHT-1 :((x + center.y < 0) ?0 :(x + center.y)), colorR, colorG, colorB);
			printPixel((-y + center.x > WIDTH-1) ?WIDTH-1 :((-y + center.x < 0) ?0 :(-y + center.x)), (x + center.y > HEIGHT-1) ?HEIGHT-1 :((x + center.y < 0) ?0 :(x + center.y)), colorR, colorG, colorB);
			printPixel((y + center.x > WIDTH-1) ?WIDTH-1 :((y + center.x < 0) ?0 :(y + center.x)), (-x + center.y > HEIGHT-1) ?HEIGHT-1 :((-x + center.y < 0) ?0 :(-x + center.y)), colorR, colorG, colorB);
			printPixel((-y + center.x > WIDTH-1) ?WIDTH-1 :((-y + center.x < 0) ?0 :(-y + center.x)), (-x + center.y > HEIGHT-1) ?HEIGHT-1 :((-x + center.y < 0) ?0 :(-x + center.y)), colorR, colorG, colorB);

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
    point p1, p2, p3, p4;
    p1 = setPoint(pivot.x-d1/2, pivot.y);
    p2 = setPoint(pivot.x, pivot.y-d2/2);
    p3 = setPoint(pivot.x+d1/2, pivot.y);
    p4 = setPoint(pivot.x, pivot.y+d2/2);

    p1 = rotatePoint(scalePoint(p1, scale_factor, pivot), theta, pivot);
    p2 = rotatePoint(scalePoint(p2, scale_factor, pivot), theta, pivot);
    p3 = rotatePoint(scalePoint(p3, scale_factor, pivot), theta, pivot);
    p4 = rotatePoint(scalePoint(p4, scale_factor, pivot), theta, pivot);

    drawLine(p1, p2, 1, colorR, colorG, colorB);
    drawLine(p2, p3, 1, colorR, colorG, colorB);
    drawLine(p3, p4, 1, colorR, colorG, colorB);
    drawLine(p4, p1, 1, colorR, colorG, colorB);
    
}

void drawAirplane(point offset, double scale_factor, double theta){
    //left wing
    point p1,p2, pcolor;
    point pivot = setPoint(150+offset.x, 90+offset.y);
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
        
        drawLine(p1, p2, 1, 107,91,0);    
    }
    //rasterize(pcolor.y-50 * scale_factor, pcolor.x, 150 * scale_factor,150 * scale_factor,107,91,0);
    
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
        
        drawLine(p1, p2, 1, 107,91,0);     
    }
    //rasterize(pcolor.y-50 * scale_factor, pcolor.x-20, 150 * scale_factor,150 * scale_factor,107,91,0);
    
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
        
        drawLine(p1, p2, 1, 107,91,0);   
    }
    //rasterize(pcolor.y-20 * scale_factor, pcolor.x-20 * scale_factor, 150 * scale_factor,150 * scale_factor,107,91,0);

    //cockpit
    p1 = setPoint(jet.cockpit_center.x + offset.x, jet.cockpit_center.y + offset.y);
    p1 = rotatePoint(scalePoint(p1,scale_factor,pivot),theta,pivot);
    drawCircle(p1, jet.cockpit_radius * scale_factor, 114,114,114);
    /*rasterize(p1.y - (jet.cockpit_radius+5) * scale_factor, 
              p1.x - (jet.cockpit_radius+5) * scale_factor, 
              2 * (jet.cockpit_radius+5) * scale_factor, 
              2 * (jet.cockpit_radius+5) * scale_factor, 
              114, 114, 114);*/

    //body
    p1 = setPoint(jet.body_center.x + offset.x, jet.body_center.y + offset.y);
    drawCircle(p1, jet.body_radius * scale_factor,168,143,0);
    /*rasterize(p1.y - (jet.body_radius+5) * scale_factor, 
              p1.x - (jet.body_radius+5) * scale_factor, 
              2 * (jet.body_radius+5) * scale_factor, 
              2 * (jet.body_radius+5) * scale_factor, 
              168, 143, 0);*/

    //inner
    p1 = setPoint(jet.inner_center.x + offset.x, jet.inner_center.y + offset.y);
    drawCircle(p1, jet.inner_radius * scale_factor,107, 80,0);
    /*rasterize(p1.y - (jet.inner_radius+5) * scale_factor, 
              p1.x - (jet.inner_radius+5) * scale_factor, 
              2 * (jet.inner_radius+5) * scale_factor, 
              2 * (jet.inner_radius+5) * scale_factor, 
              107, 80, 0);*/

    
}

