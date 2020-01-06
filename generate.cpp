#include <stdio.h >
#include <stdlib.h >
#include "math.h" 
#include <string>
#include<sstream>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <random>
#include <iostream>

#pragma comment(lib,"comctl32.lib")
#pragma comment(lib, "comctl32.lib")
#pragma comment(lib, "vfw32.lib")

#pragma comment(lib, "opencv_world412.lib")

using namespace std;

int main(void)
{
    //決定的な乱数の生成、ランダムに肉のサイズ、色、脂肪交雑の数を変更することが出来るようにする。
    std::random_device rnd;     // 非決定的な乱数生成器シード用
    std::mt19937 mt(rnd());     // メルセンヌ・ツイスタの32ビット版、引数は初期シード
    mt.seed(rnd());
    std::uniform_int_distribution<> sizerand(0, 39);

    std::uniform_int_distribution<> mrrand35(0, 34);
    std::uniform_int_distribution<> mgrand20(0, 19);
    std::uniform_int_distribution<> mbrand5(0, 4);

    std::uniform_int_distribution<> frrand3(0, 2);
    std::uniform_int_distribution<> fgrand54(0, 26);
    std::uniform_int_distribution<> fbrand158(0, 78);

    std::uniform_int_distribution<> marblrand400(299, 999);
    std::uniform_int_distribution<> disrand20(0, 19);
    int rad[4];

    // # CV_8UC3 : 8bit 3channel フルカラー画像
    cv::Size image_size(1600, 600);
    cv::Mat	image(image_size, CV_8UC3);
    cv::Mat	pimage(image_size, CV_8UC3);
    cv::Mat	turing(image_size, CV_8UC3);
    cv::RNG gen(0xffffffff);
    
    image = cv::Scalar(0, 0, 0);
    pimage = cv::Scalar(0, 0, 0);

    cv::putText(image, "Meat Generator ver2.00", cv::Point(100, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(5, 255, 5), 1);
    cv::putText(pimage, "Meat Generator ver2.00", cv::Point(100, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(5, 255, 5), 1);
    cv::putText(image, "by Kmt", cv::Point(100, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(5, 255, 5), 1);
    cv::putText(pimage, "by Kmt", cv::Point(100, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(5, 255, 5), 1);

    int sx[4], sy[4], mcr[4], mcg[4], mcb[4], fcr[4], fcg[4], fcb[4];
    int gain = sizerand(mt);
    double nbcs, nsize[4];
    //Generate randum size and color
    for (int i = 0; i < 4; i++)
    {
        rad[i] = sizerand(mt);
        int sgn = rad[i] - gain;
        if (sgn <= 0)
            sgn = -1;
        else
            sgn = 1;
        sx[i] = 240 + sgn*rad[i];

        rad[i] = sizerand(mt);
        sgn = rad[i] - gain;
        if (sgn <= 0)
            sgn = -1;
        else
            sgn = 1;
        sy[i] = 220 + sgn*rad[i];
        nsize[i] = 3.14159268*sx[i] / 2 * sy[i] / 2;

        mcr[i] = 205 + mrrand35(mt);
        mcg[i] = 49 + mgrand20(mt);
        mcb[i] = 28 + mbrand5(mt);

        nbcs = 19.858019 - 0.095460824*mcr[i] - 0.008808958*mcg[i] + 0.17291258*mcb[i];

        fcr[i] = 250 + frrand3(mt);
        fcr[i] = 220 + fgrand54(mt);
        fcr[i] = 160 + fbrand158(mt);

        cv::ellipse(image, cv::Point2f(200 + 300 * i, 200), cv::Size(sx[i] / 2, sy[i] / 2), 0, 0, 360, cv::Scalar(mcb[i], mcg[i], mcr[i]), -1, cv::LINE_AA, 0);

        std::ostringstream bcs, size;

        bcs << "BCS: " << nbcs;
        size << "Size[pt2]:  " << nsize[i];

        //putText(img,画像オブジェクト, text(書き込む文字列、string), org(書き込み位置),fontface(フォンとスタイル),fontScale(フォントサイズ), color(フォントの色), thickness(フォント太さ),lineType)
        cv::putText(image, bcs.str().c_str(), cv::Point(100 + 300 * i, 460), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
        cv::putText(pimage, bcs.str().c_str(), cv::Point(100 + 300 * i, 460), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
        cv::putText(image, size.str().c_str(), cv::Point(100 + 300 * i, 490), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
        cv::putText(pimage, size.str().c_str(), cv::Point(100 + 300 * i, 490), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    }

    int dislan, dislan2, sgn, marbnum, marbx, marby, fatpar, fatarea, fatcount[4];
    double rootinside, elceny, xd, yd, xsq, ysq, bms;

    for (int meat = 0; meat < 4; meat++)
    {
        std::uniform_int_distribution<> mrblrandx(200 + (300 * meat) - abs(sx[meat] / 2), 200 + (300 * meat) + abs(sx[meat] / 2));
        std::uniform_int_distribution<> mrblrandy(200 - abs(sy[meat] / 2), 200 + abs(sy[meat] / 2));
        std::uniform_int_distribution<> szrand(1, 6);

        marbnum = marblrand400(mt);

        std::ostringstream n2s;    

        fatarea, fatcount[meat] = 0;

        for (int plot = 0; plot < marbnum; plot++)
        {
            marbx = mrblrandx(mt);
            marby = mrblrandy(mt);

            elceny = marby - 200;

            rootinside = (sy[meat] * sy[meat] / 4) - (elceny*elceny);

            if (rootinside > 0)
                rootinside = (sx[meat] * sx[meat] / 4) - ((sx[meat] * sx[meat] / 4) / (sy[meat] * sy[meat]) / 4)*(elceny*elceny);

            xd = marbx - 200 - 300 * meat;
            yd = marby - 200;
            xsq = xd*xd / (sx[meat] * sx[meat] / 4);
            ysq = yd*yd / (sy[meat] * sy[meat] / 4);
            dislan = disrand20(mt);
            dislan2 = disrand20(mt);
            if (dislan - dislan2 < 0)
                sgn = -1;
            else
                sgn = 1;

            if (xsq + ysq <= 1)
            {
                fatpar = szrand(mt);

                if (xsq + ysq <= 0.49)
                    cv::circle(image, cv::Point2f(marbx, marby), fatpar, cv::Scalar(255, 255, 255), -1, -1, 0);
                else
                    cv::circle(image, cv::Point2f(marbx, marby), fatpar, cv::Scalar(255, 255, 255), -1, -1, 0);
                fatarea = fatpar*fatpar*3.141592;
                fatcount[meat] = fatcount[meat] + fatarea;
            }
        }
        //口田先生の1999年の論文より参照。BMSSUBは重回帰で0.98の決定係数をあらわしている。
        bms = -1.26 + 100 * 0.462*fatcount[meat] / nsize[meat];
        n2s << "BMS:  " << bms;
        //putText(img,画像オブジェクト, text(書き込む文字列、string), org(書き込み位置),fontface(フォンとスタイル),fontScale(フォントサイズ), color(フォントの色), thickness(フォント太さ),lineType)
        cv::putText(image, n2s.str().c_str(), cv::Point(100 + 300 * meat, 520), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    }

    //
    float lu,cu,ru,lc,cc,rc,ld,cd,rd,d1,d2,f,k;
    lu=1;
    cu=1;
    ru=1;
    lc=1;
    cc=-8;
    rc=1;
    ld=1;
    cd=1;
    rd=1;

    d1=0.02;
    d2=0.5;
    f=0.00;
    k=0.063;

    int counter=0;

    //このフィルタは間違い　要修正
    for(int meat=0;meat<4;meat++)
    {
        for(int meaty=201-sy[meat]/2;meaty<199+sy[meat]/2;meaty++)
        {
            for(int meatx=201+300*meat-sx[meat]/2;meatx<199+300*meat+sx[meat]/2;meatx++)
            {
                for(int step=0;step<2;step++)
                {
                    cv::Vec3b pixv,pixu,u,v,plu,pcu,pru,plc,pcc,prc,pld,pcd,prd;
                    pixu=image.at<cv::Vec3b>(meaty,meatx);
                    pixv=image.at<cv::Vec3b>(meaty,meatx);
                    u=image.at<cv::Vec3b>(meaty,meatx);
                    v=image.at<cv::Vec3b>(meaty,meatx);

                    plu=image.at<cv::Vec3b>(meaty-1,meatx-1);
                    pcu=image.at<cv::Vec3b>(meaty-1,meatx);
                    pru=image.at<cv::Vec3b>(meaty-1,meatx+1);
                    plc=image.at<cv::Vec3b>(meaty,meatx-1);
                    pcc=image.at<cv::Vec3b>(meaty,meatx);
                    prc=image.at<cv::Vec3b>(meaty,meatx+1);
                    pld=image.at<cv::Vec3b>(meaty+1,meatx-1);
                    pcd=image.at<cv::Vec3b>(meaty+1,meatx);
                    prd=image.at<cv::Vec3b>(meaty+1,meatx+1);

                    pixu[0]=d1*(lu*plu[0]+cu*pcu[0]+ru*pru[0]+lc*plc[0]+cc*pcc[0]+rc*prc[0]+ld*pld[0]+cd*pcd[0]+rd*prd[0]);
                    pixu[1]=d1*(lu*plu[1]+cu*pcu[1]+ru*pru[1]+lc*plc[1]+cc*pcc[1]+rc*prc[1]+ld*pld[1]+cd*pcd[1]+rd*prd[1]);
                    pixu[2]=d1*(lu*plu[2]+cu*pcu[2]+ru*pru[2]+lc*plc[2]+cc*pcc[2]+rc*prc[2]+ld*pld[2]+cd*pcd[2]+rd*prd[2]);
                    
                   pixv[0]=d2*(lu*plu[0]+cu*pcu[0]+ru*pru[0]+lc*plc[0]+cc*pcc[0]+rc*prc[0]+ld*pld[0]+cd*pcd[0]+rd*prd[0]);
                    pixv[1]=d2*(lu*plu[1]+cu*pcu[1]+ru*pru[1]+lc*plc[1]+cc*pcc[1]+rc*prc[1]+ld*pld[1]+cd*pcd[1]+rd*prd[1]);
                    pixv[2]=d2*(lu*plu[2]+cu*pcu[2]+ru*pru[2]+lc*plc[2]+cc*pcc[2]+rc*prc[2]+ld*pld[2]+cd*pcd[2]+rd*prd[2]);
                    pixu[0]=pixu[0]-(u[0]*v[0]*v[0])+f*(1.0-u[0]);
                    pixu[1]=pixu[1]-(u[1]*v[1]*v[1])+f*(1.0-u[1]);
                    pixu[2]=pixu[2]-(u[2]*v[2]*v[2])+f*(1.0-u[2]);

                    pixv[0]=pixv[0]+(u[0]*v[0]*v[0])-(f+k)*v[0];
                    pixv[1]=pixv[1]+(u[1]*v[1]*v[1])-(f+k)*v[1];
                    pixv[2]=pixv[2]+(u[2]*v[2]*v[2])-(f+k)*v[2];

                    turing.at<cv::Vec3b>(meaty,meatx)=pixu;
                    counter++; 
                }
            }
        }
    }
    std::ostringstream n2s;    
    
    for(int meat=0;meat<4;meat++)
    {
    int fcount=0;
    std::ostringstream n2s;     
        cv::ellipse(pimage, cv::Point2f(200 + 300 * meat, 200), cv::Size(sx[meat] / 2, sy[meat] / 2), 0, 0, 360, cv::Scalar(mcb[meat], mcg[meat], mcr[meat]), -1, cv::LINE_AA, 0);
        for(int meaty=201-sy[meat]/2;meaty<199+sy[meat]/2;meaty++)
        {
            for(int meatx=201+300*meat-sx[meat]/2;meatx<199+300*meat+sx[meat]/2;meatx++)
            {
                xd = meatx - 200 - 300 * meat;
                yd = meaty - 200;
                xsq = xd*xd / (sx[meat] * sx[meat] / 4);
                ysq = yd*yd / (sy[meat] * sy[meat] / 4);

                if (xsq + ysq <= 1)
                {
                    cv::Vec3b pix;
                    pix=turing.at<cv::Vec3b>(meaty,meatx);

                    if(pix[0]<20&&pix[1]<20&&pix[2]<20)
                    {
                        pix[0]=255;
                        pix[1]=255;
                        pix[2]=255;
                        pimage.at<cv::Vec3b>(meaty,meatx)=pix;
                        fcount++;
                    }
                }               
            }
        }
        bms = -1.26 + 100 * 0.462*(fcount) /( 3.141592*(sx[meat]/2)*(sy[meat]/2));
        n2s << "BMS:  " << bms;

        cv::putText(pimage, n2s.str().c_str(), cv::Point(100 + 300 * meat, 520), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    }

    cv::imshow("perfect", pimage);
    cv::waitKey();
    return 0;

}
