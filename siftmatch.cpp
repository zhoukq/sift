#include "siftmatch.h"
#include "ui_siftmatch.h"

#include <QtCore>
#include <QtGui>
#include <QFile>
#include <QTextStream>

//SIFT算法头文件
//加extern "C"，告诉编译器按C语言的方式编译和连接
extern "C"
{
#include "imgfeatures.h"
#include "kdtree.h"
#include "minpq.h"
#include "sift.h"
#include "utils.h"
#include "xform.h"
}

//在k-d树上进行BBF搜索的最大次数
/* the maximum number of keypoint NN candidates to check during BBF search */
#define KDTREE_BBF_MAX_NN_CHKS 1000

//目标点与最近邻和次近邻的距离的比值的阈值，若大于此阈值，则剔除此匹配点对
//通常此值取0.6，值越小找到的匹配点对越精确，但匹配数目越少
/* threshold on squared ratio of distances between NN and 2nd NN */
#define NN_SQ_DIST_RATIO_THR 0.6

//窗口名字符串
#define IMG0 "pic0"
#define IMG1 "pic1"
#define IMG2 "pic2"
#define IMG3 "pic3"
#define IMG0_FEAT "pic0 feature"
#define IMG1_FEAT "pic1 feature"
#define IMG2_FEAT "pic2 feature"
#define IMG3_FEAT "pic3 feature"
#define IMG_MATCH1 "distance match1"
#define IMG_MATCH2 "distance match2"
#define IMG_MATCH3 "distance match3"
#define IMG_MATCH1 "RANSAC match1"
#define IMG_MATCH2 "RANSAC match2"
#define IMG_MATCH3 "RANSAC match3"
#define IMG_MOSAIC_TEMP "MOSAIC_TEMP"
#define IMG_MOSAIC_SIMPLE "MOSAIC_SIMPLE"
#define IMG_MOSAIC_BEFORE_FUSION "MOSAIC_BEFORE_FUSION"
#define IMG_MOSAIC_PROC "MOSAIC_PROC"


SiftMatch::SiftMatch(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::SiftMatch)
{    
    open_image_number = 0;//打开图片的个数

    //设定横竖排的单选按钮的勾选状态，默认是横排
    for(int i=0;i<4;i++)
    {
        img[i]=NULL;
        img_Feat[i]=NULL;
    }
    for(int i=0;i<3;i++)
    {
        stacked[i]=NULL;
        stacked_ransac[i]=NULL;
        H[i]=NULL;
    }

    QFile ini_file("/home/zhou/sift_match/data.ini");

    if(ini_file.open(QIODevice::ReadOnly|QIODevice::Text))
    {
        QTextStream txtInput(&ini_file);
        QString pos;
        int j=0;
        while(!txtInput.atEnd())
        {
            for(int i=0;i<4;i++)
            {
                pos=txtInput.readLine().section("=",1);
                //qDebug()<<pos<<endl;
                a[j][i]=pos.toDouble();
            }
            j++;
        }
        ini_file.close();
    }
    else
    {
        qDebug()<<"read error"<<endl;
    }

    xformed = NULL;

    verticalStackFlag = false;//显示匹配结果的合成图像默认是横向排列

    ui->setupUi(this);

    ui->openButton->setText("first pic");

    //禁用下列按钮
    ui->detectButton->setEnabled(false);
    ui->radioButton_horizontal->setEnabled(false);
    ui->radioButton_vertical->setEnabled(false);
    ui->matchButton->setEnabled(false);
    ui->mosaicButton->setEnabled(false);
    ui->restartButton->setEnabled(false);
}

SiftMatch::~SiftMatch()
{
    delete ui;
}

//打开图片
void SiftMatch::on_openButton_clicked()
{
//    double a1 = -0.74698780106517149, a2= 2.3387246230570700, a3=-4.4404083231844647;
//    double B01data[] = {7.3278474649451425e+002, 0., 280., 0.,7.5630160640330973e+002, 209., 0., 0., 1.};
//    Mat cameraMatrix = Mat(3,3,CV_64F,B01data).clone();
//    double B10data[] = {a1, a2, 0., 0.,a3,0,0,0};
//    Mat distCoeffs = Mat(8,1,CV_64F,B10data).clone();
    name[0]="pic1.bmp";
    name[1]="pic2.bmp";
    name[2]="pic3.bmp";
    name[3]="pic4.bmp";
    write_file();
    img[0]=cvLoadImage("/home/zhou/pic/21:55:47.bmp");
    img[1]=cvLoadImage("/home/zhou/pic/22:01:58.bmp");
    img[2]=cvLoadImage("/home/zhou/pic/22:02:55.bmp");
    img[3]=cvLoadImage("/home/zhou/pic/22:03:40.bmp");

//    img[0]=cvLoadImage("/home/zhou/cv/jpg/bmp/20141025155308.bmp");
//    img[1]=cvLoadImage("/home/zhou/cv/jpg/bmp/20141025155313.bmp");
//    img[2]=cvLoadImage("/home/zhou/cv/jpg/bmp/20141025155316.bmp");
//    img[3]=cvLoadImage("/home/zhou/cv/jpg/bmp/20141025155318.bmp");
//    Mat srcc(img[0]),dstt=srcc.clone();
//    undistort(srcc, dstt, cameraMatrix, distCoeffs,cameraMatrix);
    ui->openButton->setText("stop");//改变按钮文本
    ui->openButton->setEnabled(false);
    ui->detectButton->setEnabled(true);//激活特征检测按钮
    //只能打开2张图片
//    if(open_image_number <= 3 )
//    {
//        //注意：文件目录中不能有中文，两张图片的文件名最好为：xxx1.jpg,xxx2.jpg
//        //一般情况下xxx1.jpg应是左图，xxx2.jpg应是右图，但如果反了，程序可自动适应
//        QString img_name = QFileDialog::getOpenFileName(this,"Open Image", QDir::currentPath(),tr("Image Files(*.png *.jpeg *.jpg *.bmp *.gif)"));
//        if(!img_name.isNull())
//        {
//            open_image_number++;
//            //打开第1张图片
//            if( 1 == open_image_number )
//            {
//                ui->openButton->setText("second pic");//改变按钮文本
//                ui->restartButton->setEnabled(true);//激活重选按钮

//                name[0] = img_name;//保存第1张图片的文件名
//                //qDebug()<<name1.insert( name1.lastIndexOf(".",-1) , "_Feat");
//                img[0] = cvLoadImage( img_name.toAscii().data() );//打开图1，强制读取为三通道图像
//                qDebug()<<img_name.toAscii().data()<<endl;
//                //cvNamedWindow(IMG0);//创建窗口
//                //cvShowImage(IMG0,img[0]);//显示原图1
//            }
//            //打开第2张图片
//            else if( 2 == open_image_number )
//            {
//                ui->openButton->setText("third pic");//改变按钮文本
//                ui->openButton->setEnabled(true);
//                //ui->detectButton->setEnabled(true);//激活特征检测按钮

//                name[1] = img_name;//保存第2张图片的文件名
//                img[1] = cvLoadImage( img_name.toAscii().data() );//打开图2，强制读取为三通道图像
//                //cvNamedWindow(IMG1);//创建窗口
//                //cvShowImage(IMG1,img[1]);//显示原图2
//            }
//            else if( 3 == open_image_number )
//            {
//                ui->openButton->setText("forth pic");//改变按钮文本
//                ui->openButton->setEnabled(true);
//                //ui->detectButton->setEnabled(true);//激活特征检测按钮

//                name[2] = img_name;//保存第2张图片的文件名
//                img[2] = cvLoadImage( img_name.toAscii().data() );//打开图2，强制读取为三通道图像
//                //cvNamedWindow(IMG2);//创建窗口
//                //cvShowImage(IMG2,img[2]);//显示原图2
//            }
//            else if( 4 == open_image_number )
//            {
//                ui->openButton->setText("stop");//改变按钮文本
//                ui->openButton->setEnabled(false);
//                ui->detectButton->setEnabled(true);//激活特征检测按钮

//                name[3] = img_name;//保存第2张图片的文件名
//                img[3] = cvLoadImage( img_name.toAscii().data() );//打开图2，强制读取为三通道图像
//                //cvNamedWindow(IMG3);//创建窗口
//                //cvShowImage(IMG3,img[3]);//显示原图2
//            }
//        }
//    }
}

//特征点检测
void SiftMatch::on_detectButton_clicked()
{
    //img[0]_Feat = cvCloneImage(img[0]);//复制图1，深拷贝，用来画特征点
    //img2_Feat = cvCloneImage(img2);//复制图2，深拷贝，用来画特征点
    for(int i=0;i<4;i++)
    {
        img_Feat[i] = cvCloneImage(img[i]);
        //默认提取的是LOWE格式的SIFT特征点
        //提取并显示第i幅图片上的特征点
        n[i] = sift_features( img[i], &feat[i] );//检测图1中的SIFT特征点,n1是图1的特征点个数
        //export_features("feature1.txt",feat1,n1);//将特征向量数据写入到文件
        draw_features( img_Feat[i], feat[i], n[i] );//画出特征点
        //cvNamedWindow(QString::number(i,10).toLatin1().data());//创建窗口
        //cvShowImage(QString::number(i,10).toLatin1().data(),img_Feat[i]);//显示
        QString name1_Feat = name[0];//文件名，原文件名加"_Feat"
        cvSaveImage(name1_Feat.insert( name1_Feat.lastIndexOf(".",-1) , "_Feat").toAscii().data(),img_Feat[i]);//保存图片
    }


    ui->detectButton->setEnabled(false);//禁用特征检测按钮
    ui->radioButton_horizontal->setEnabled(true);//激活排列方向选择按钮
    ui->radioButton_vertical->setEnabled(true);
    ui->matchButton->setEnabled(true);//激活特征匹配按钮
}

//特征匹配
void SiftMatch::on_matchButton_clicked()
{
    //若用户勾选了水平排列按钮
    if(ui->radioButton_horizontal->isChecked())
    {
        //将2幅图片合成1幅图片,img[0]在左，img2在右
        for(int i=0;i<3;i++)
        {
            stacked[i] = stack_imgs_horizontal(img[i], img[i+1]);//合成图像，显示经距离比值法筛选后的匹配结果
        }

    }
    else//用户勾选了垂直排列按钮
    {
        verticalStackFlag = true;//垂直排列标识设为true
        //将2幅图片合成1幅图片,img[0]在上，img2在下
        for(int i=0;i<3;i++)
        {
            stacked[i] = stack_imgs(img[i], img[i+1]);//合成图像，显示经距离比值法筛选后的匹配结果
        }

    }


    Point pt1,pt2;//连线的两个端点
    double d0,d1;//feat2中每个特征点到最近邻和次近邻的距离
    int matchNum = 0;//经距离比值法筛选后的匹配点对的个数

    for(int ii=1;ii<4;ii++)
    {
        matchNum = 0;

        //根据图1的特征点集feat1建立k-d树，返回k-d树根给kd_root
        kd_root = kdtree_build( feat[ii-1], n[ii-1] );

        //遍历特征点集feat2，针对feat2中每个特征点feat，选取符合距离比值条件的匹配点，放到feat的fwd_match域中
        for(int i = 0; i < n[ii]; i++ )
        {
            feats = feat[ii]+i;//第i个特征点的指针
            //在kd_root中搜索目标点feat的2个最近邻点，存放在nbrs中，返回实际找到的近邻点个数
            int k = kdtree_bbf_knn( kd_root, feats, 2, &nbrs, KDTREE_BBF_MAX_NN_CHKS );
            if( k == 2 )
            {
                d0 = descr_dist_sq( feats, nbrs[0] );//feat与最近邻点的距离的平方
                d1 = descr_dist_sq( feats, nbrs[1] );//feat与次近邻点的距离的平方
                //若d0和d1的比值小于阈值NN_SQ_DIST_RATIO_THR，则接受此匹配，否则剔除
                if( d0 < d1 * NN_SQ_DIST_RATIO_THR )
                {   //将目标点feat和最近邻点作为匹配点对
                    pt2 = Point( cvRound( feats->x ), cvRound( feats->y ) );//图2中点的坐标
                    pt1 = Point( cvRound( nbrs[0]->x ), cvRound( nbrs[0]->y ) );//图1中点的坐标(feat的最近邻点)
                    if(verticalStackFlag)//垂直排列
                        pt2.y += img[ii-1]->height;//由于两幅图是上下排列的，pt2的纵坐标加上图1的高度，作为连线的终点
                    else
                        pt2.x += img[ii-1]->width;//由于两幅图是左右排列的，pt2的横坐标加上图1的宽度，作为连线的终点
                    cvLine( stacked[ii-1], pt1, pt2, CV_RGB(255,0,255), 1, 8, 0 );//画出连线
                    matchNum++;//统计匹配点对的个数
                    feat[ii][i].fwd_match = nbrs[0];//使点feat的fwd_match域指向其对应的匹配点
                }
            }
            free( nbrs );//释放近邻数组
        }
        qDebug()<<"distance num"<<matchNum<<endl;
    }



    //显示并保存经距离比值法筛选后的匹配图
//    cvNamedWindow(IMG_MATCH1);//创建窗口
//    cvShowImage(IMG_MATCH1,stacked[0]);//显示
//    cvNamedWindow(IMG_MATCH2);//创建窗口
//    cvShowImage(IMG_MATCH2,stacked[1]);//显示
//    cvNamedWindow(IMG_MATCH3);//创建窗口
//    cvShowImage(IMG_MATCH3,stacked[2]);//显示
    //保存匹配图
    QString name_match_DistRatio = name[0];//文件名，原文件名去掉序号后加"_match_DistRatio"
    cvSaveImage(name_match_DistRatio.replace( name_match_DistRatio.lastIndexOf(".",-1)-1 , 1 , "_match_DistRatio").toAscii().data(),stacked[0]);
    cvSaveImage(name_match_DistRatio.replace( name_match_DistRatio.lastIndexOf(".",-1)-1 , 1 , "_match_DistRatio").toAscii().data(),stacked[1]);
    cvSaveImage(name_match_DistRatio.replace( name_match_DistRatio.lastIndexOf(".",-1)-1 , 1 , "_match_DistRatio").toAscii().data(),stacked[2]);



    for(int j=0;j<3;j++)
    {
        //利用RANSAC算法筛选匹配点,计算变换矩阵H，
        //无论img[0]和img2的左右顺序，H永远是将feat2中的特征点变换为其匹配点，即将img2中的点变换为img[0]中的对应点
        H[j] = ransac_xform(feat[j+1],n[j+1],FEATURE_FWD_MATCH,lsq_homog,4,0.01,homog_xfer_err,3.0,&inliers,&n_inliers);

        cvmSet(H[j],0,0,1);
        cvmSet(H[j],0,1,0);
        cvmSet(H[j],1,0,0);
        cvmSet(H[j],1,1,1);
        cvmSet(H[j],2,0,0);
        cvmSet(H[j],2,1,0);

        //若能成功计算出变换矩阵，即两幅图中有共同区域
        if( H[j] )
        {
            qDebug()<<"RANSAC num :"<<n_inliers<<endl;

           //输出H矩阵
           for(int i=0;i<3;i++)
                qDebug()<<cvmGet(H[j],i,0)<<cvmGet(H[j],i,1)<<cvmGet(H[j],i,2);

            if(verticalStackFlag)//将2幅图片合成1幅图片,img[0]在上，img2在下
                stacked_ransac[j] = stack_imgs( img[j], img[j+1] );//合成图像，显示经RANSAC算法筛选后的匹配结果
            else//将2幅图片合成1幅图片,img[0]在左，img2在右
                stacked_ransac[j] = stack_imgs_horizontal(img[j], img[j+1] );//合成图像，显示经RANSAC算法筛选后的匹配结果

            //img[0]LeftBound = inliers[0]->fwd_match->x;//图1中匹配点外接矩形的左边界
            //img[0]RightBound = img[0]LeftBound;//图1中匹配点外接矩形的右边界
            //img2LeftBound = inliers[0]->x;//图2中匹配点外接矩形的左边界
            //img2RightBound = img2LeftBound;//图2中匹配点外接矩形的右边界

            int invertNum = 0;//统计pt2.x > pt1.x的匹配点对的个数，来判断img[0]中是否右图
            float xsum=0,ysum=0;
            //遍历经RANSAC算法筛选后的特征点集合inliers，找到每个特征点的匹配点，画出连线
            for(int i=0; i<n_inliers; i++)
            {
                feats = inliers[i];//第i个特征点
                pt2 = Point(cvRound(feats->x), cvRound(feats->y));//图2中点的坐标
                pt1 = Point(cvRound(feats->fwd_match->x), cvRound(feats->fwd_match->y));//图1中点的坐标(feat的匹配点)
                //qDebug()<<"pt2:("<<pt2.x<<","<<pt2.y<<")--->pt1:("<<pt1.x<<","<<pt1.y<<")";//输出对应点对

                xsum=xsum+pt2.x-pt1.x;
                ysum=ysum+pt2.y-pt1.y;
                /*找匹配点区域的边界
                if(pt1.x < img[0]LeftBound) img[0]LeftBound = pt1.x;
                if(pt1.x > img[0]RightBound) img[0]RightBound = pt1.x;
                if(pt2.x < img2LeftBound) img2LeftBound = pt2.x;
                if(pt2.x > img2RightBound) img2RightBound = pt2.x;//*/

                //统计匹配点的左右位置关系，来判断图1和图2的左右位置关系
                if(pt2.x > pt1.x)
                    invertNum++;

                if(verticalStackFlag)//垂直排列
                    pt2.y += img[j]->height;//由于两幅图是上下排列的，pt2的纵坐标加上图1的高度，作为连线的终点
                else//水平排列
                    pt2.x += img[j]->width;//由于两幅图是左右排列的，pt2的横坐标加上图1的宽度，作为连线的终点
                cvLine(stacked_ransac[j],pt1,pt2,CV_RGB(255,0,255),1,8,0);//在匹配图上画出连线
            }

            qDebug()<<xsum/n_inliers<<endl<<ysum/n_inliers<<endl;
            cvmSet(H[j],0,2,0-xsum/n_inliers);
            cvmSet(H[j],1,2,0-ysum/n_inliers);

            //绘制图1中包围匹配点的矩形
            //cvRectangle(stacked_ransac,cvPoint(img[0]LeftBound,0),cvPoint(img[0]RightBound,img[0]->height),CV_RGB(0,255,0),2);
            //绘制图2中包围匹配点的矩形
            //cvRectangle(stacked_ransac,cvPoint(img[0]->width+img2LeftBound,0),cvPoint(img[0]->width+img2RightBound,img2->height),CV_RGB(0,0,255),2);

            cvNamedWindow(QString::number(j*10,10).toLatin1().data());//创建窗口
            cvShowImage(QString::number(j*10,10).toLatin1().data(),stacked_ransac[j]);//显示经RANSAC算法筛选后的匹配图
            //保存匹配图
//            QString name_match_RANSAC = name[0];//文件名，原文件名去掉序号后加"_match_RANSAC"
//            cvSaveImage(name_match_RANSAC.replace( name_match_RANSAC.lastIndexOf(".",-1)-1 , 1 , "_match_RANSAC").toAscii().data(),stacked_ransac[j]);


            /*程序中计算出的变换矩阵H用来将img2中的点变换为img[0]中的点，正常情况下img[0]应该是左图，img2应该是右图。
              此时img2中的点pt2和img[0]中的对应点pt1的x坐标的关系基本都是：pt2.x < pt1.x
              若用户打开的img[0]是右图，img2是左图，则img2中的点pt2和img[0]中的对应点pt1的x坐标的关系基本都是：pt2.x > pt1.x
              所以通过统计对应点变换前后x坐标大小关系，可以知道img[0]是不是右图。
              如果img[0]是右图，将img[0]中的匹配点经H的逆阵H_IVT变换后可得到img2中的匹配点*/

            //若pt2.x > pt1.x的点的个数大于内点个数的80%，则认定img[0]中是右图
            /*
            if(invertNum > n_inliers * 0.8)
            {
                //qDebug()<<tr("img[0]中是右图");
                CvMat * H_IVT = cvCreateMat(3, 3, CV_64FC1);//变换矩阵的逆矩阵
                //求H的逆阵H_IVT时，若成功求出，返回非零值
                if( cvInvert(H,H_IVT) )
                {
    //                //输出H_IVT
    //                for(int i=0;i<3;i++)
    //                    qDebug()<<cvmGet(H_IVT,i,0)<<cvmGet(H_IVT,i,1)<<cvmGet(H_IVT,i,2);
                    cvReleaseMat(&H);//释放变换矩阵H，因为用不到了
                    H = cvCloneMat(H_IVT);//将H的逆阵H_IVT中的数据拷贝到H中
                    cvReleaseMat(&H_IVT);//释放逆阵H_IVT
                    //将img[0]和img2对调
                    IplImage * temp = img2;
                    img2 = img[0];
                    img[0] = temp;
                    //cvShowImage(img[0],img[0]);
                    //cvShowImage(IMG2,img2);
                    ui->mosaicButton->setEnabled(true);//激活全景拼接按钮
                }
                else//H不可逆时，返回0
                {
                    cvReleaseMat(&H_IVT);//释放逆阵H_IVT
                    QMessageBox::warning(this,tr("警告"),tr("变换矩阵H不可逆"));
                }
            }
            else*/
                ui->mosaicButton->setEnabled(true);//激活全景拼接按钮
        }
        else //无法计算出变换矩阵，即两幅图中没有重合区域
        {
            QMessageBox::warning(this,"warning","no common area");
        }
    }

    ui->radioButton_horizontal->setEnabled(false);//禁用排列方向选择按钮
    ui->radioButton_vertical->setEnabled(false);
    ui->matchButton->setEnabled(false);//禁用特征匹配按钮
}

//计算图2的四个角经矩阵H变换后的坐标
void SiftMatch::CalcFourCorner()
{
    double v2[]={0,0,1};//左上角
    double v1[3];//变换后的坐标值

    for(int i=0;i<3;i++)
    {
        //计算图2的四个角经矩阵H变换后的坐标//        cvmSet(H[j],0,0,1);
        //        cvmSet(H[j],0,1,0);
        //        cvmSet(H[j],1,0,0);
        //        cvmSet(H[j],1,1,1);
        //        cvmSet(H[j],2,0,0);
        //        cvmSet(H[j],2,1,0);

        CvMat V2 = cvMat(3,1,CV_64FC1,v2);
        CvMat V1 = cvMat(3,1,CV_64FC1,v1);
        for(int j=i;j>=0;j--)
        {
            cvGEMM(H[j],&V2,1,0,1,&V1);//矩阵乘法
            v2[0]=cvRound(v1[0]/v1[2]);
            v2[1]=cvRound(v1[1]/v1[2]);
            v2[2]=1;
            V2=cvMat(3,1,CV_64FC1,v2);
        }
        leftTop[i].x = cvRound(v1[0]/v1[2]);
        leftTop[i].y = cvRound(v1[1]/v1[2]);
        //cvCircle(xformed,leftTop,7,CV_RGB(255,0,0),2);

        //将v2中数据设为左下角坐标
        v2[0] = 0;
        v2[1] = img[i+1]->height;
        V2 = cvMat(3,1,CV_64FC1,v2);
        V1 = cvMat(3,1,CV_64FC1,v1);
        for(int j=i;j>=0;j--)
        {
            cvGEMM(H[j],&V2,1,0,1,&V1);//矩阵乘法
            v2[0]=cvRound(v1[0]/v1[2]);
            v2[1]=cvRound(v1[1]/v1[2]);
            v2[2]=1;
            V2=cvMat(3,1,CV_64FC1,v2);
        }
        leftBottom[i].x = cvRound(v1[0]/v1[2]);
        leftBottom[i].y = cvRound(v1[1]/v1[2]);
        //cvCircle(xformed,leftBottom,7,CV_RGB(255,0,0),2);

        //将v2中数据设为右上角坐标
        v2[0] = img[i+1]->width;
        v2[1] = 0;
        V2 = cvMat(3,1,CV_64FC1,v2);
        V1 = cvMat(3,1,CV_64FC1,v1);
        for(int j=i;j>=0;j--)
        {
            cvGEMM(H[j],&V2,1,0,1,&V1);//矩阵乘法
            v2[0]=cvRound(v1[0]/v1[2]);
            v2[1]=cvRound(v1[1]/v1[2]);
            v2[2]=1;
            V2=cvMat(3,1,CV_64FC1,v2);
        }
        rightTop[i].x = cvRound(v1[0]/v1[2]);
        rightTop[i].y = cvRound(v1[1]/v1[2]);
        //cvCircle(xformed,rightTop,7,CV_RGB(255,0,0),2);

        //将v2中数据设为右下角坐标
        v2[0] = img[i+1]->width;
        v2[1] = img[i+1]->height;
        V2 = cvMat(3,1,CV_64FC1,v2);
        V1 = cvMat(3,1,CV_64FC1,v1);
        for(int j=i;j>=0;j--)
        {
            cvGEMM(H[j],&V2,1,0,1,&V1);//矩阵乘法
            v2[0]=cvRound(v1[0]/v1[2]);
            v2[1]=cvRound(v1[1]/v1[2]);
            v2[2]=1;
            V2=cvMat(3,1,CV_64FC1,v2);//        cvmSet(H[j],0,0,1);
            //        cvmSet(H[j],0,1,0);
            //        cvmSet(H[j],1,0,0);
            //        cvmSet(H[j],1,1,1);
            //        cvmSet(H[j],2,0,0);
            //        cvmSet(H[j],2,1,0);
        }
        rightBottom[i].x = cvRound(v1[0]/v1[2]);
        rightBottom[i].y = cvRound(v1[1]/v1[2]);
        //cvCircle(xformed,rightBottom,7,CV_RGB(255,0,0),2);
        //qDebug()<<rightBottom[i].x<<" "<<rightBottom[i].y<<endl;
        //qDebug()<<rightTop[i].x<<" "<<rightTop[i].y<<endl;
    }


}

//全景拼接
void SiftMatch::on_mosaicButton_clicked()
{
    CvMat *T0=cvCloneMat(H[0]),*T1=cvCloneMat(H[1]),*T2=cvCloneMat(H[2]);
    //若能成功计算出变换矩阵，即两幅图中有共同区域，才可以进行全景拼接
    if(H)
    {
        //拼接图像，img[0]是左图，img2是右图
        CalcFourCorner();//计算图2的四个角经变换后的坐标
        cvGEMM(T1,T0,1,0,1,T1);
        cvGEMM(T2,T1,1,0,1,T2);

        //为拼接结果图xformed分配空间,高度为图1图2高度的较小者，根据图2右上角和右下角变换后的点的位置决定拼接图的宽度
        xformed = cvCreateImage(cvSize(MIN(rightTop[2].x,rightBottom[2].x),MIN(MIN(MIN(img[0]->height,img[1]->height),img[2]->height),img[3]->height)),IPL_DEPTH_8U,3);
        IplImage *xformed1=cvCloneImage(xformed),*xformed2=cvCloneImage(xformed);
        //用变换矩阵H对右图img2做投影变换(变换后会有坐标右移)，结果放到xformed中
        cvWarpPerspective(img[1],xformed,T0,CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS,cvScalarAll(0));
        cvWarpPerspective(img[2],xformed1,T1,CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS,cvScalarAll(0));
        cvWarpPerspective(img[3],xformed2,T2,CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS,cvScalarAll(0));

//        cvNamedWindow("IMG_MOSAIC_SIMPLE1");//创建窗口
//        cvShowImage("IMG_MOSAIC_SIMPLE1",xformed);//显示简易拼接图
//        cvNamedWindow("IMG_MOSAIC_SIMPLE2");//创建窗口
//        cvShowImage("IMG_MOSAIC_SIMPLE2",xformed1);//显示简易拼接图
//        cvNamedWindow("IMG_MOSAIC_SIMPLE3");//创建窗口
//        cvShowImage("IMG_MOSAIC_SIMPLE3",xformed2);//显示简易拼接图

//        cvSaveImage("c1.bmp",xformed);
//        cvSaveImage("c2.bmp",xformed1);
//        cvSaveImage("c3.bmp",xformed2);

        cvSetImageROI(xformed1,cvRect(MIN(leftTop[1].x,leftBottom[1].x),0,MIN(rightTop[1].x,rightBottom[1].x)-MIN(leftTop[1].x,leftBottom[1].x),MIN(MIN(MIN(img[0]->height,img[1]->height),img[2]->height),img[3]->height)));
        cvSetImageROI(xformed,cvRect(MIN(leftTop[1].x,leftBottom[1].x),0,MIN(rightTop[1].x,rightBottom[1].x)-MIN(leftTop[1].x,leftBottom[1].x),MIN(MIN(MIN(img[0]->height,img[1]->height),img[2]->height),img[3]->height)));
        cvAddWeighted(xformed1,1,xformed,0,0,xformed);
        cvResetImageROI(xformed);
        cvResetImageROI(xformed1);

        cvSetImageROI(xformed2,cvRect(MIN(leftTop[2].x,leftBottom[2].x),0,MIN(rightTop[2].x,rightBottom[2].x)-MIN(leftTop[2].x,leftBottom[2].x),MIN(MIN(MIN(img[0]->height,img[1]->height),img[2]->height),img[3]->height)));
        cvSetImageROI(xformed,cvRect(MIN(leftTop[2].x,leftBottom[2].x),0,MIN(rightTop[2].x,rightBottom[2].x)-MIN(leftTop[2].x,leftBottom[2].x),MIN(MIN(MIN(img[0]->height,img[1]->height),img[2]->height),img[3]->height)));
        cvAddWeighted(xformed2,1,xformed,0,0,xformed);
        cvResetImageROI(xformed);
        cvResetImageROI(xformed2);


        cvSetImageROI(xformed,cvRect(0,0,MIN(leftTop[0].x,leftBottom[0].x),xformed->height));
        cvSetImageROI(img[0],cvRect(0,0,MIN(leftTop[0].x,leftBottom[0].x),xformed->height));
        cvAddWeighted(img[0],1,xformed,0,0,xformed);
        cvResetImageROI(xformed);

        cvNamedWindow(IMG_MOSAIC_SIMPLE,CV_WINDOW_NORMAL);//创建窗口
        cvShowImage(IMG_MOSAIC_SIMPLE,xformed);//显示简易拼接图


//        cvSaveImage("call.bmp",xformed);

        /*
        //简易拼接法：直接将将左图img[0]叠加到xformed的左边
        xformed_simple = cvCloneImage(xformed);//简易拼接图，可笼子xformed
        cvSetImageROI(xformed_simple,cvRect(0,0,img[0]->width,img[0]->height));
        cvAddWeighted(img[0],1,xformed_simple,0,0,xformed_simple);
        cvResetImageROI(xformed_simple);
        cvNamedWindow(IMG_MOSAIC_SIMPLE);//创建窗口
        cvShowImage(IMG_MOSAIC_SIMPLE,xformed_simple);//显示简易拼接图

        //处理后的拼接图，克隆自xformed
        xformed_proc = cvCloneImage(xformed);

        //重叠区域左边的部分完全取自图1
        cvSetImageROI(img[0],cvRect(0,0,MIN(leftTop.x,leftBottom.x),xformed_proc->height));
        cvSetImageROI(xformed,cvRect(0,0,MIN(leftTop.x,leftBottom.x),xformed_proc->height));
        cvSetImageROI(xformed_proc,cvRect(0,0,MIN(leftTop.x,leftBottom.x),xformed_proc->height));
        cvAddWeighted(img[0],1,xformed,0,0,xformed_proc);
        cvResetImageROI(img[0]);
        cvResetImageROI(xformed);
        cvResetImageROI(xformed_proc);
        cvNamedWindow(IMG_MOSAIC_BEFORE_FUSION);
        cvShowImage(IMG_MOSAIC_BEFORE_FUSION,xformed_proc);//显示融合之前的拼接图

        //采用加权平均的方法融合重叠区域
        int start = MIN(leftTop.x,leftBottom.x) ;//开始位置，即重叠区域的左边界
        double processWidth = img[0]->width - start;//重叠区域的宽度
        double alpha = 1;//img[0]中像素的权重
        for(int i=0; i<xformed_proc->height; i++)//遍历行
        {
            const uchar * pixel_img[0] = ((uchar *)(img[0]->imageData + img[0]->widthStep * i));//img[0]中第i行数据的指针
            const uchar * pixel_xformed = ((uchar *)(xformed->imageData + xformed->widthStep * i));//xformed中第i行数据的指针
            uchar * pixel_xformed_proc = ((uchar *)(xformed_proc->imageData + xformed_proc->widthStep * i));//xformed_proc中第i行数据的指针
            for(int j=start; j<img[0]->width; j++)//遍历重叠区域的列
            {
                //如果遇到图像xformed中无像素的黑点，则完全拷贝图1中的数据
                if(pixel_xformed[j*3] < 50 && pixel_xformed[j*3+1] < 50 && pixel_xformed[j*3+2] < 50 )
                {
                    alpha = 1;
                }
                else
                {   //img[0]中像素的权重，与当前处理点距重叠区域左边界的距离成正比
                    alpha = (processWidth-(j-start)) / processWidth ;
                }
                pixel_xformed_proc[j*3] = pixel_img[0][j*3] * alpha + pixel_xformed[j*3] * (1-alpha);//B通道
                pixel_xformed_proc[j*3+1] = pixel_img[0][j*3+1] * alpha + pixel_xformed[j*3+1] * (1-alpha);//G通道
                pixel_xformed_proc[j*3+2] = pixel_img[0][j*3+2] * alpha + pixel_xformed[j*3+2] * (1-alpha);//R通道
            }
        }
        cvNamedWindow(IMG_MOSAIC_PROC);//创建窗口
        cvShowImage(IMG_MOSAIC_PROC,xformed_proc);//显示处理后的拼接图

        //重叠区域取两幅图像的平均值，效果不好
        //设置ROI，是包含重叠区域的矩形
        /*
        cvSetImageROI(xformed_proc,cvRect(MIN(leftTop.x,leftBottom.x),0,img[0]->width-MIN(leftTop.x,leftBottom.x),xformed_proc->height));
        cvSetImageROI(img[0],cvRect(MIN(leftTop.x,leftBottom.x),0,img[0]->width-MIN(leftTop.x,leftBottom.x),xformed_proc->height));
        cvSetImageROI(xformed,cvRect(MIN(leftTop.x,leftBottom.x),0,img[0]->width-MIN(leftTop.x,leftBottom.x),xformed_proc->height));
        cvAddWeighted(img[0],0.5,xformed,0.5,0,xformed_proc);
        cvResetImageROI(xformed_proc);
        cvResetImageROI(img[0]);
        cvResetImageROI(xformed); //*/

        /*对拼接缝周围区域进行滤波来消除拼接缝，效果不好
        //在处理前后的图上分别设置横跨拼接缝的矩形ROI
        cvSetImageROI(xformed_proc,cvRect(img[0]->width-10,0,img[0]->width+10,xformed->height));
        cvSetImageROI(xformed,cvRect(img[0]->width-10,0,img[0]->width+10,xformed->height));
        cvSmooth(xformed,xformed_proc,CV_MEDIAN,5);//对拼接缝周围区域进行中值滤波
        cvResetImageROI(xformed);
        cvResetImageROI(xformed_proc);
        cvShowImage(IMG_MOSAIC_PROC,xformed_proc);//显示处理后的拼接图 */

        /*想通过锐化解决变换后的图像失真的问题，对于扭曲过大的图像，效果不好
        double a[]={  0, -1,  0, -1,  5, -1, 0, -1,  0  };//拉普拉斯滤波核的数据
        CvMat kernel = cvMat(3,3,CV_64FC1,a);//拉普拉斯滤波核
        cvFilter2D(xformed_proc,xformed_proc,&kernel);//滤波
        cvShowImage(IMG_MOSAIC_PROC,xformed_proc);//显示处理后的拼接图

        //保存拼接图
        QString name_xformed = name[0];//文件名，原文件名去掉序号后加"_Mosaic"
        cvSaveImage(name_xformed.replace( name_xformed.lastIndexOf(".",-1)-1 , 1 , "_Mosaic").toAscii().data() , xformed_simple);//保存简易拼接图
        cvSaveImage(name_xformed.insert( name_xformed.lastIndexOf(".",-1) , "_Proc").toAscii().data() , xformed_proc);//保存处理后的拼接图
        ui->mosaicButton->setEnabled(false);//禁用全景拼接按钮
        */
    }

}

// 重新选择
void SiftMatch::on_restartButton_clicked()
{
    /*
    //释放并关闭原图1
    if(img[0])
    {
        cvReleaseImage(&img[0]);
        cvDestroyWindow(IMG1);
    }
    //释放并关闭原图2
    if(img[1])
    {
        cvReleaseImage(&img2);
        cvDestroyWindow(IMG2);
    }
    //释放特征点图1
    if(img_Feat[0])
    {
        cvReleaseImage(&img_Feat);
        cvDestroyWindow(IMG1_FEAT);
        free(feat1);//释放特征点数组
    }
    //释放特征点图2
    if(img_Feat[1])
    {
        cvReleaseImage(&img_Feat);
        cvDestroyWindow(IMG2_FEAT);
        free(feat2);//释放特征点数组
    }
    //释放距离比值筛选后的匹配图和kd树
    if(stacked[0])
    {
        cvReleaseImage(&stacked);
        cvDestroyWindow(IMG_MATCH1);
        kdtree_release(kd_root);//释放kd树
    }
    //只有在RANSAC算法成功算出变换矩阵时，才需要进一步释放下面的内存空间
    if( H )
    {
        cvReleaseMat(&H);//释放变换矩阵H
        free(inliers);//释放内点数组

        //释放RANSAC算法筛选后的匹配图
        cvReleaseImage(&stacked_ransac);
        cvDestroyWindow(IMG_MATCH2);

        //释放全景拼接图像
        if(xformed)
        {
            cvReleaseImage(&xformed);
            cvReleaseImage(&xformed_simple);
            cvReleaseImage(&xformed_proc);
            cvDestroyWindow(IMG_MOSAIC_TEMP);
            cvDestroyWindow(IMG_MOSAIC_SIMPLE);
            cvDestroyWindow(IMG_MOSAIC_BEFORE_FUSION);
            cvDestroyWindow(IMG_MOSAIC_PROC);
        }
    }

    open_image_number = 0;//打开图片个数清零
    verticalStackFlag = false;//显示匹配结果的合成图片的排列方向标识复位

    ui->openButton->setEnabled(true);//激活打开按钮
    ui->openButton->setText("first pic");

    //禁用下列按钮
    ui->detectButton->setEnabled(false);
    ui->radioButton_horizontal->setEnabled(false);
    ui->radioButton_vertical->setEnabled(false);
    ui->matchButton->setEnabled(false);
    ui->mosaicButton->setEnabled(false);
    ui->restartButton->setEnabled(false);
    */
}



void SiftMatch::on_k1addButton_clicked()
{
    int seq=0;
    if(ui->radioButton_pic0->isChecked())
    {
        seq=0;
    }
    else if(ui->radioButton_pic1->isChecked())
    {
        seq=1;
    }
    else if(ui->radioButton_pic2->isChecked())
    {
        seq=2;
    }
    else if(ui->radioButton_pic3->isChecked())
    {
        seq=3;
    }

    a[0][seq]=a[0][seq]+0.05;
    distort_process();
}

void SiftMatch::on_k2addButton_clicked()
{
    int seq=0;
    if(ui->radioButton_pic0->isChecked())
    {
        seq=0;
    }
    else if(ui->radioButton_pic1->isChecked())
    {
        seq=1;
    }
    else if(ui->radioButton_pic2->isChecked())
    {
        seq=2;
    }
    else if(ui->radioButton_pic3->isChecked())
    {
        seq=3;
    }

    a[1][seq]=a[1][seq]+0.05;
    distort_process();
}

void SiftMatch::on_k3addButton_clicked()
{
    int seq=0;
    if(ui->radioButton_pic0->isChecked())
    {
        seq=0;
    }
    else if(ui->radioButton_pic1->isChecked())
    {
        seq=1;
    }
    else if(ui->radioButton_pic2->isChecked())
    {
        seq=2;
    }
    else if(ui->radioButton_pic3->isChecked())
    {
        seq=3;
    }

    a[2][seq]=a[2][seq]+0.05;
    distort_process();
}

void SiftMatch::on_k1deButton_clicked()
{
    int seq=0;
    if(ui->radioButton_pic0->isChecked())
    {
        seq=0;
    }
    else if(ui->radioButton_pic1->isChecked())
    {
        seq=1;
    }
    else if(ui->radioButton_pic2->isChecked())
    {
        seq=2;
    }
    else if(ui->radioButton_pic3->isChecked())
    {
        seq=3;
    }

    a[0][seq]=a[0][seq]-0.05;
    distort_process();
}

void SiftMatch::on_k2deButton_clicked()
{
    int seq=0;
    if(ui->radioButton_pic0->isChecked())
    {
        seq=0;
    }
    else if(ui->radioButton_pic1->isChecked())
    {
        seq=1;
    }
    else if(ui->radioButton_pic2->isChecked())
    {
        seq=2;
    }
    else if(ui->radioButton_pic3->isChecked())
    {
        seq=3;
    }

    a[1][seq]=a[1][seq]-0.05;
    distort_process();
}

void SiftMatch::on_k3deButton_clicked()
{
    int seq=0;
    if(ui->radioButton_pic0->isChecked())
    {
        seq=0;
    }
    else if(ui->radioButton_pic1->isChecked())
    {
        seq=1;
    }
    else if(ui->radioButton_pic2->isChecked())
    {
        seq=2;
    }
    else if(ui->radioButton_pic3->isChecked())
    {
        seq=3;
    }

    a[2][seq]=a[2][seq]-0.05;
    distort_process();
}

void SiftMatch::distort_process()
{



    CvMat *T0=cvCloneMat(H[0]),*T1=cvCloneMat(H[1]),*T2=cvCloneMat(H[2]);
    double B01data[] = {7.3278474649451425e+002, 0., 280., 0.,7.5630160640330973e+002, 209., 0., 0., 1.};
    Mat cameraMatrix = Mat(3,3,CV_64F,B01data).clone();
    double B10data[] = {a[0][0], a[1][0], 0., 0.,a[2][0],0,0,0};
    double B12data[] = {a[0][1], a[1][1], 0., 0.,a[2][1],0,0,0};
    double B23data[] = {a[0][2], a[1][2], 0., 0.,a[2][2],0,0,0};
    double B34data[] = {a[0][3], a[1][3], 0., 0.,a[2][3],0,0,0};
    Mat distCoeffs0 = Mat(8,1,CV_64F,B10data).clone(),distCoeffs1 = Mat(8,1,CV_64F,B12data).clone(),distCoeffs2 = Mat(8,1,CV_64F,B23data).clone(),distCoeffs3 = Mat(8,1,CV_64F,B34data).clone();
    Mat srcc0(img[0]),dstt0=srcc0.clone();
    undistort(srcc0, dstt0, cameraMatrix, distCoeffs0,cameraMatrix);
    Mat srcc1(img[1]),dstt1=srcc1.clone();
    undistort(srcc1, dstt1, cameraMatrix, distCoeffs1,cameraMatrix);
    Mat srcc2(img[2]),dstt2=srcc2.clone();
    undistort(srcc2, dstt2, cameraMatrix, distCoeffs2,cameraMatrix);
    Mat srcc3(img[3]),dstt3=srcc3.clone();
    undistort(srcc3, dstt3, cameraMatrix, distCoeffs3,cameraMatrix);
    IplImage img0(dstt0),img1(dstt1),img2(dstt2),img3(dstt3);
    write_file();
    cvDestroyAllWindows();
    if(H)
    {
        //拼接图像，img[0]是左图，img2是右图
        CalcFourCorner();//计算图2的四个角经变换后的坐标
        cvGEMM(T1,T0,1,0,1,T1);
        cvGEMM(T2,T1,1,0,1,T2);

        //为拼接结果图xformed分配空间,高度为图1图2高度的较小者，根据图2右上角和右下角变换后的点的位置决定拼接图的宽度
        xformed = cvCreateImage(cvSize(MIN(rightTop[2].x,rightBottom[2].x),MIN(MIN(MIN(img[0]->height,img[1]->height),img[2]->height),img[3]->height)),IPL_DEPTH_8U,3);
        IplImage *xformed1=cvCloneImage(xformed),*xformed2=cvCloneImage(xformed);
        //用变换矩阵H对右图img2做投影变换(变换后会有坐标右移)，结果放到xformed中

        cvWarpPerspective(&img1,xformed,T0,CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS,cvScalarAll(0));
        cvWarpPerspective(&img2,xformed1,T1,CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS,cvScalarAll(0));
        cvWarpPerspective(&img3,xformed2,T2,CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS,cvScalarAll(0));

        cvSetImageROI(xformed1,cvRect(MIN(leftTop[1].x,leftBottom[1].x),0,MIN(rightTop[1].x,rightBottom[1].x)-MIN(leftTop[1].x,leftBottom[1].x),MIN(MIN(MIN(img[0]->height,img[1]->height),img[2]->height),img[3]->height)));
        cvSetImageROI(xformed,cvRect(MIN(leftTop[1].x,leftBottom[1].x),0,MIN(rightTop[1].x,rightBottom[1].x)-MIN(leftTop[1].x,leftBottom[1].x),MIN(MIN(MIN(img[0]->height,img[1]->height),img[2]->height),img[3]->height)));
        cvAddWeighted(xformed1,1,xformed,0,0,xformed);
        cvResetImageROI(xformed);
        cvResetImageROI(xformed1);

        cvSetImageROI(xformed2,cvRect(MIN(leftTop[2].x,leftBottom[2].x),0,MIN(rightTop[2].x,rightBottom[2].x)-MIN(leftTop[2].x,leftBottom[2].x),MIN(MIN(MIN(img[0]->height,img[1]->height),img[2]->height),img[3]->height)));
        cvSetImageROI(xformed,cvRect(MIN(leftTop[2].x,leftBottom[2].x),0,MIN(rightTop[2].x,rightBottom[2].x)-MIN(leftTop[2].x,leftBottom[2].x),MIN(MIN(MIN(img[0]->height,img[1]->height),img[2]->height),img[3]->height)));
        cvAddWeighted(xformed2,1,xformed,0,0,xformed);
        cvResetImageROI(xformed);
        cvResetImageROI(xformed2);

        cvSetImageROI(xformed,cvRect(0,0,MIN(leftTop[0].x,leftBottom[0].x),xformed->height));
        cvSetImageROI(&img0,cvRect(0,0,MIN(leftTop[0].x,leftBottom[0].x),xformed->height));
        cvAddWeighted(&img0,1,xformed,0,0,xformed);
        cvResetImageROI(xformed);

//        cvSetImageROI(xformed,cvRect(0,0,img0.width,img0.height));
//        cvSetImageROI(&img0,cvRect(0,0,img0.width,img0.height));
//        cvAddWeighted(&img0,1,xformed,0,0,xformed);
//        cvResetImageROI(xformed);
//        cvResetImageROI(&img0);

//        cvSetImageROI(xformed,cvRect(cvmGet(H[0],0,2),cvmGet(H[0],1,2)<0?0:cvmGet(H[0],1,2),img1.width,cvmGet(H[0],1,2)<0?img1.height+cvmGet(H[0],1,2)<0:img1.height-cvmGet(H[0],1,2)));
//        cvSetImageROI(&img1,cvRect(0,cvmGet(H[0],1,2)<0?-cvmGet(H[0],1,2):0,img1.width,cvmGet(H[0],1,2)<0?img1.height+cvmGet(H[0],1,2):img1.height-cvmGet(H[0],1,2)));
//        cvAddWeighted(&img1,1,xformed,0,0,xformed);
//        cvResetImageROI(xformed);
//        cvResetImageROI(&img1);

        cvNamedWindow("IMG_MOSAIC_SIMPLE");//创建窗口
        cvMoveWindow("IMG_MOSAIC_SIMPLE",0,0);
        cvShowImage("IMG_MOSAIC_SIMPLE",xformed);//显示简易拼接图
    }

}

void SiftMatch::write_file()
{
    QFile ini_file("/home/zhou/sift_match/data.ini");

    if(ini_file.open(QIODevice::WriteOnly|QIODevice::Text))
    {
        QTextStream txtInput(&ini_file);
        for(int j=0;j<3;j++)
        {
            for(int i=0;i<4;i++)
            {
                txtInput<<"a"<<j<<i<<"="<<a[j][i]<<endl;
            }
        }
        ini_file.close();
    }
    else
    {
        qDebug()<<"read error"<<endl;
    }
}
