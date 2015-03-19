#ifndef SIFTMATCH_H
#define SIFTMATCH_H

#include <QDialog>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;

namespace Ui {
class SiftMatch;
}

class SiftMatch : public QDialog
{
    Q_OBJECT
    
public:
    explicit SiftMatch(QWidget *parent = 0);
    ~SiftMatch();

    void CalcFourCorner();//计算图2的四个角经矩阵H变换后的坐标
    
private slots:
    void on_openButton_clicked();

    void on_detectButton_clicked();

    void on_matchButton_clicked();

    void on_restartButton_clicked();

    void on_mosaicButton_clicked();


    void on_k1addButton_clicked();

    void on_k2addButton_clicked();

    void on_k3addButton_clicked();

    void on_k1deButton_clicked();

    void on_k2deButton_clicked();

    void on_k3deButton_clicked();

    void distort_process();

    void write_file();

private:
    Ui::SiftMatch *ui;

    int open_image_number;//打开图片个数
    QString name[4];//两张图片的文件名
    IplImage *img[4];//IplImage格式的原图
    IplImage *img_Feat[4];//画上特征点之后的图

    bool verticalStackFlag;//显示匹配结果的合成图像中，两张图是纵向排列的标志
    IplImage *stacked[3];//显示匹配结果的合成图像，显示经距离比值法筛选后的匹配结果
    IplImage *stacked_ransac[3];//显示匹配结果的合成图像，显示经RANSAC算法筛选后的匹配结果

    struct feature *feat[4];//feat1：图1的特征点数组，feat2：图2的特征点数组
    int n[4];//n1:图1中的特征点个数，n2：图2中的特征点个数
    struct feature *feats;//每个特征点
    struct kd_node *kd_root;//k-d树的树根
    struct feature **nbrs;//当前特征点的最近邻点数组

    CvMat * H[3];//RANSAC算法求出的变换矩阵
    struct feature **inliers;//精RANSAC筛选后的内点数组
    int n_inliers;//经RANSAC算法筛选后的内点个数,即feat2中具有符合要求的特征点的个数

    IplImage *xformed;//临时拼接图，即只将图2变换后的图
    IplImage *xformed_simple;//简易拼接图
    IplImage *xformed_proc;//处理后的拼接图
    double a[3][4] = {{-0.74698780106517149,-0.74698780106517149,-0.74698780106517149,-0.74698780106517149},{2.3387246230570700,2.3387246230570700,2.3387246230570700,2.3387246230570700},{-4.4404083231844647,-4.4404083231844647,-4.4404083231844647,-4.4404083231844647}};
    //double a2[4]= {2.3387246230570700,2.3387246230570700,2.3387246230570700,2.3387246230570700};
    //double a3[4]={-4.4404083231844647,-4.4404083231844647,-4.4404083231844647,-4.4404083231844647};


//    int img1LeftBound;//图1中匹配点外接矩形的左边界
//    int img1RightBound;//图1中匹配点外接矩形的右边界
//    int img2LeftBound;//图2中匹配点外接矩形的左边界
//    int img2RightBound;//图2中匹配点外接矩形的右边界

    //图2的四个角经矩阵H变换后的坐标
    CvPoint leftTop[3],leftBottom[3],rightTop[3],rightBottom[3];


};

#endif // SIFTMATCH_H
