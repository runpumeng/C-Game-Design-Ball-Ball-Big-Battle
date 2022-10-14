#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "fssimplewindow.h"
#include <math.h>
#include <time.h>
#include "ysglfontdata.h"



const double PI = 3.1415926;
const int nBall = 30;   // the obstacle balls count


void DrawPlayerBall(double x,double y,double r)   // draw the player's ball
{
    glBegin(GL_POLYGON);
    int sqn = 3600;
    for(int i=0;i<sqn;i++)
    {
        glVertex2f(x+r*10.0*cos(2*PI*i/sqn), y+r*10.0*sin(2*PI*i/sqn));
    }
    glEnd();
}



int CheckHitEdge(double ballx,double bally,double r)  // check if it touch the edge
{
     
    if(ballx-r*10<0 || ballx+r*10>800 || bally-r*10<0 || bally+r*10>600)
    {
        return 0;
    }
    else
    {
        return 1;
    }
}



void DrawCircle(int cx,int cy,int rad,int fill,int state)  //draw the obstacle ball
{
    const double YS_PI=3.1415927;

    if(0!=fill && state == 1)
    {
        glBegin(GL_POLYGON);
    }

    int i;
    for(i=0; i<64; i++)
    {
        double angle=(double)i*YS_PI/32.0;
        double x=(double)cx+cos(angle)*(double)rad;
        double y=(double)cy+sin(angle)*(double)rad;
        glVertex2d(x,y);
    }

    glEnd();
}




void Move(double &x,double &y,double &vx,double &vy,double m,double dt)
{
    double fx,fy,ax,ay;

    x=x+vx*dt;
    y=y+vy*dt;

    fx=0.0;
    fy=-m*9.8;  // Gravity

    ax=fx/m;
    ay=fy/m;

    vx=vx+dt*ax;
    vy=vy+dt*ay;
}

void Bounce(double &x,double &y,double &vx,double &vy,double &br)
{
    if(y<br && vy<0.0)
    {
        vy=-vy;
    }

    if(x<-(40-br) && vx<0.0)
    {
        vx=-vx;
    }
    else if(x>(40-br) && vx>0.0)
    {
        vx=-vx;
    }
}




void CalculateCollision(
    double x1,double y1,double &vx1,double &vy1,
    double x2,double y2,double &vx2,double &vy2,
    double radius1, double radius2)
{
    double dx=x2-x1,dy=y2-y1;
    double d=dx*dx+dy*dy;
    if(d<(radius1*2.0)*(radius2*2.0))
    {
        double nx,ny,nl;
        nx=x2-x1;
        ny=y2-y1;

        nl=nx*nx+ny*ny;
        if(0.0<nl)
        {
            nl=sqrt(nl);
            nx/=nl;
            ny/=nl;

            double nv1,nv2;
            nv1=(vx1*nx+vy1*ny);
            nv2=(vx2*nx+vy2*ny);

            if((nv2>0.0 && nv2<nv1) ||
               (nv1<0.0 && nv2<nv1) ||
               (nv1>0.0 && nv2<0.0))
            {
                vx1=vx1-nx*nv1+nx*nv2;
                vy1=vy1-ny*nv1+ny*nv2;

                vx2=vx2-nx*nv2+nx*nv1;
                vy2=vy2-ny*nv2+ny*nv1;
            }
        }
    }
}



void CollisionDetection(int n,double x[],double y[],double vx[],double vy[],double br[])
{
    int i,j;
    for(i=0; i<n; i++)
    {
        for(j=i+1; j<n; j++)
        {
            CalculateCollision(
                x[i],y[i],vx[i],vy[i],
                x[j],y[j],vx[j],vy[j],br[i],br[j]);
        }
    }
}

void SwapInt(int &a,int &b)
{
    int c;
    c=b;
    b=a;
    a=c;
}


// check if the player's ball hit the obstacle ball
int CheckHitObstacle(int mx,int my,double x,double y, double r, double br)
{
    double relativeD;
    relativeD=((double)mx-(400+x*10.0))*((double)mx-(400+x*10.0))+((double)my-(600-y*10.0))*((double)my-(600-y*10.0));
    relativeD = sqrt(relativeD);
    if((r+br)*10 >= relativeD)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

// draw the obstacle rectangular
void DrawObstacle(double init_x,double init_y,double height,double width)
{

    glColor3ub(0,255,0);
    glBegin(GL_QUADS);
    glVertex2i(init_x,init_y);
    glVertex2i(init_x + width,init_y);
    glVertex2i(init_x+ width,init_y+height);
    glVertex2i(init_x,init_y+ height);
    glEnd();
    
}

// check if the player's ball hit the rectangular
int CheckHitObstacle2(int mx,int my,double ObstacleX,double ObstacleY,double width,double height,double r)
{
    double relativeX1,relativeY1,relativeX2,relativeY2;
    relativeX1=(double)mx-ObstacleX;
    relativeY1=(double)my-ObstacleY;
    relativeX2=ObstacleX-(double)mx;
    relativeY2=ObstacleY-(double)my;
    if(relativeX1>=0 && relativeX1<width && relativeY1 == height+r*10) //bottom
    {
        return 1;
    }
    else if(relativeX1>=0 && relativeX1<width && relativeY2 == r*10) // top
    {
        return 1;
    }
    else if(relativeX2 == r*10 && relativeY1>=0 && relativeY1 < height) // left
    {
        return 1;
    }
    else if(relativeX1 == width+r*10 && relativeY1>=0 && relativeY1 < height) // right
    {
        return 1;
    }
    else
    {
        return 0;
    }
}






int main(void)
{
    FsOpenWindow(0,0,800,600,1);
    srand(time(NULL));
    
    int t = 50;
    int i;
    double m[nBall],x[nBall],y[nBall],vx[nBall],vy[nBall],dt,br[nBall];
    double init_x[5]; // initial the 5 random obstacle
    double init_y[5];
    double range1[5];
    double range2[5];
    double height[5];
    double width[5];
    
    
    for(int i=0;i<5;i++){
        init_x[i] = rand()% 700;
        init_y[i] = rand()% 500;
        range1[i] = rand()% 50;
        range2[i] = rand()% 50;
        height[i] = range1[i] + 50.0;
        width[i] = range2[i] + 50.0;
    }
    
    
    for(i=0; i<nBall; i++)
        {
            br[i]=(double)(rand()%7);   // obstacle ball's radius
            m[i]=1.0*PI*br[i]*br[i];

            x[i]=(double)(rand()%80-(40-br[i]));  // obstacle ball's position
            y[i]=(double)(rand()%40+10*br[i]);

            vx[i]=(double)(rand()%40-20);
            vy[i]=0.0;
            
        }
    dt=0.02;
    double r = 3;
    

    int terminal =0;
    int ball = 0;     // ball=0 represent the ball influenced by size limit
    int num = 0;
    int prenum = 0;
    int invinc = 1;   // invinc = 1 means you only have 1 chance(state) to use                      invincible skill and eat 3 smaller ball and disfunction
    int obstacle[nBall];
    for(int i=0;i<nBall;i++){
        obstacle[i]=1;
    }
    int plcolor[3];
    
    plcolor[0]=rand()%2;    // random change the player ball's color
    plcolor[1]=rand()%2;
    plcolor[2]=rand()%2;
    if(plcolor[0] == plcolor[1] == plcolor[2] == 1){
        int select = rand()%3;
        plcolor[select] = 0;
    }
    
    int blcolor[nBall];    ///  use shuffle to get a random order and then random give color
    for(int i=0;i<nBall;i++){
        blcolor[i]=i;
    }
    
    for(int j=0;j<nBall;j++){
        int k = rand()%nBall;
        SwapInt(blcolor[j],blcolor[k]);
    }
    
    int a = rand()%20;
    int b = rand()%20;
    int c = rand()%20;
    

    
    
    while(0 == terminal)
    {
        FsPollDevice();
        auto key=FsInkey();
        if(FSKEY_ESC==key)
        {
            break;
        }
        

        int lb,mb,rb,mx,my;
        int evt=FsGetMouseEvent(lb,mb,rb,mx,my);

        if(FSMOUSEEVENT_LBUTTONDOWN==evt)
        {
            glClearColor(0,0,1,0);
        }
        else if(FSMOUSEEVENT_LBUTTONUP==evt)
        {
            glClearColor(0,1,1,0);
        }
        else
        {
            glClearColor(1,1,1,0);
        }
        
        
        
        
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
        
        
        glColor3ub(plcolor[0],plcolor[1],plcolor[2]);
       
        
        DrawPlayerBall(mx, my, r);
        
        
        //  draw the triangle obstacle and check if the player ball hit it
        for(int i = 0;i<5;i++){
            DrawObstacle(init_x[i],init_y[i],height[i],width[i]);
        }
        
        
        for(int i=0;i<5;i++)
        {
        CheckHitObstacle2(mx, my, init_x[i], init_y[i], width[i], height[i],r);
        if(CheckHitObstacle2(mx, my, init_x[i], init_y[i], width[i], height[i],r)==1)
        {
                terminal = 1;
                printf("Sorry,you hit the obstacle, you lose!!!");
            }
        }
        
        
        // check the player ball hit the edge of the window
        CheckHitEdge(mx, my,r);
        if(CheckHitEdge(mx,my,r)==0)
             {
                 printf("Hit the edg, you lose!\n");
                 terminal=1;
             }
        
        
        // draw the moving obstacle ball
        for(i=0; i<nBall; i++)
        {
            glColor3ub(blcolor[i]*a,blcolor[i]*b,blcolor[i]*c);
            DrawCircle(400+(x[i]*10.0),600-y[i]*10.0,br[i]*10,1,obstacle[i]);
        }
        
        
        // let the moving ball bounce and check if they touch each other
        for(i=0; i<nBall; i++)
            {
                Move(x[i],y[i],vx[i],vy[i],m[i],dt);
                Bounce(x[i],y[i],vx[i],vy[i],br[i]);
            }
        CollisionDetection(nBall,x,y,vx,vy,br);


        // check if the player ball hit the bigger one or smaller one
        for(int i=0;i<nBall;i++){
            if(CheckHitObstacle(mx, my, x[i], y[i], r, br[i])==1 && r<br[i] && ball==0)
            {
                printf("You hit the bigger obstacle ball, you lose!");
                terminal=1;

            }
            if(obstacle[i]!=0){
            if(CheckHitObstacle(mx, my, x[i], y[i], r, br[i])==1 && r>br[i])
            {
                obstacle[i] = 0;
                r += br[i]/10;
                num++;
                plcolor[0] = blcolor[i]*a;
                plcolor[1] = blcolor[i]*b;
                plcolor[2] = blcolor[i]*c;
                
            }
        }
        }
        
        if(FSKEY_SPACE==key)  /// press the space_key can let the player's ball size -> half of it
        {
            r = r/2.0;
        }
        
        if(invinc == 1){
        if(FSKEY_B==key)  /// press B to become invincible but it can still eat small and not die
        {
            ball =1;
            prenum = num;
            invinc = 0;
        }
        }

        if(num-prenum > 1) ///  But the invinvible has limitation if it eat 2 balls it will disappear
        {
            ball = 0;
        }
        
        
        glColor3f(0,0,0);
        glRasterPos2i(550,50);    // this is for the counting time(30s)
        YsGlDrawFontBitmap12x16("Surviving Time:");
        switch(t)
        {
        case 1000:
            YsGlDrawFontBitmap12x16("30");
            break;
        case 2000:
            YsGlDrawFontBitmap12x16("29");
            break;
        case 3000:
            YsGlDrawFontBitmap12x16("28");
            break;
        case 4000:
            YsGlDrawFontBitmap12x16("27");
            break;
        case 5000:
            YsGlDrawFontBitmap12x16("26");
            break;
        case 6000:
            YsGlDrawFontBitmap12x16("25");
            break;
        case 7000:
            YsGlDrawFontBitmap12x16("24");
            break;
        case 8000:
            YsGlDrawFontBitmap12x16("23");
            break;
        case 9000:
            YsGlDrawFontBitmap12x16("22");
            break;
        case 10000:
            YsGlDrawFontBitmap12x16("21");
            break;
        case 11000:
            YsGlDrawFontBitmap12x16("20");
            break;
        case 12000:
            YsGlDrawFontBitmap12x16("19");
            break;
        case 13000:
            YsGlDrawFontBitmap12x16("18");
            break;
        case 14000:
            YsGlDrawFontBitmap12x16("17");
            break;
        case 15000:
            YsGlDrawFontBitmap12x16("16");
            break;
        case 16000:
            YsGlDrawFontBitmap12x16("15");
            break;
        case 17000:
            YsGlDrawFontBitmap12x16("14");
            break;
        case 18000:
            YsGlDrawFontBitmap12x16("13");
            break;
        case 19000:
            YsGlDrawFontBitmap12x16("12");
            break;
        case 20000:
            YsGlDrawFontBitmap12x16("11");
            break;
        }
        
        // when the count to 10s, it will be red to alert
        glColor3f(1,0,0);
        glRasterPos2i(730,50);
        switch (t)
        {
        case 21000:
            YsGlDrawFontBitmap12x16("10");
            break;
        case 22000:
            YsGlDrawFontBitmap12x16("9");
            break;
        case 23000:
            YsGlDrawFontBitmap12x16("8");
            break;
        case 24000:
            YsGlDrawFontBitmap12x16("7");
            break;
        case 25000:
            YsGlDrawFontBitmap12x16("6");
            break;
        case 26000:
            YsGlDrawFontBitmap12x16("5");
            break;
        case 27000:
            YsGlDrawFontBitmap12x16("4");
            break;
        case 28000:
            YsGlDrawFontBitmap12x16("3");
            break;
        case 29000:
            YsGlDrawFontBitmap12x16("2");
            break;
        case 30000:
            YsGlDrawFontBitmap12x16("1");
            break;
        }
        
        
        t += 50;
        if(t == 30050){
            if(r>=10){
            printf("YOU HANG ON 0.5 MINUTE, YOU WIN!!!");
            terminal = 1;
            }
            else{
                printf("Sorry, You lose!!! Your size doesn't meet the request!");
                terminal = 1;
            }
        }
        
        
        FsSwapBuffers();
        FsSleep(50);
        
    }

    return 0;
}

 

