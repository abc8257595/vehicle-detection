#include <opencv.hpp>
#include <highgui/highgui.hpp>
#include <legacy/compat.hpp>
#include <stdio.h>

#define debug   //凡用ifdef debug括起来的都是测试用的
//crop:复制src内的roi_rect方形区域到dst，由于此函数使用频繁，定义成宏
#define crop(src,dst,roi_rect) ({ 		\
	cvSetImageROI((src),(roi_rect));	\
	cvCopy((src),(dst));				\
	cvResetImageROI(src);				\
})

/////////////////////////////////////－－子函数区－－///////////////////////////////////////////////////////////
//step1:灰度图的阈值分割算法

//车的阴影第一次阈值分割 shadowBound
//================================================================// 
// 功能： 从roi图像返回一个合适的阈值 
// 参数： roi 
// （入口） roi:roi图像
// （出口） thresh_1: 返回一浮点型阈值  
// 主要思路：thresh_1 = E1 - D1/a   其中E1为roi区域的灰度均值，D1为方差，a = E1 - D1
// 日期: 2014.03.28
//================================================================// 
double shadowBound(const IplImage* roi)
{
	CvScalar mean;
	CvScalar dev;

	cvAvgSdv(roi,&mean,&dev);

	double a = mean.val[0]/dev.val[0];
	double thresh_1 = mean.val[0]-dev.val[0]/a;
	return (thresh_1);
}

//车的阴影第二次阈值分割，在低于thresh_1的像素点中继续提取
//================================================================// 
// 功能： 从roi图像返回二次阈值 
// 参数： 
// （入口） roi:roi图像
// （出口） thresh_2: 返回一浮点型阈值  
//  mask : 掩码，将原255的图像转为0，用这个图像作掩码，用于cvAvgSdv
// 主要思路：第一次阈值排除图像中高亮点的干扰，二次分割统计低于threshold1
//           的点，再次计算均值方差，二次分割后可更好地抑制噪声
// 参考资料：《基于车底阴影的前方运动车辆检测》－2.1基于改进的阴影分割
// 日期: 2014.03.28
//================================================================// 
double shadowBound2(const IplImage* roi)
{
	IplImage* mask = cvCreateImage(cvGetSize(roi),IPL_DEPTH_8U,1);

	cvThreshold(roi,mask,254,255,CV_THRESH_TOZERO_INV);

	CvScalar mean;
	CvScalar dev;
	cvAvgSdv(roi,&mean,&dev,mask);
	cvReleaseImage(&mask);

	double b = mean.val[0]/dev.val[0];
	double thresh_2 = mean.val[0]-dev.val[0]/b;

	return (thresh_2);
}

//================================================================// 
// 功能： 大于阈值取max_value(255)黑点,小于则保持原值,cvThreshold刚好没这功能
// 参数： 
// （入口） src: 源图像
//	    threshold: 阈值
//	    max_value: 高于阈值所取的最大值
// （出口） dst: 目标图像
//  (内参)  psrc: src指针，指向源图像数据
//	    pdst: dst指针，指向目标图像数据
// 返回： 函数正确返回0
// 主要思路：
//	遍历整幅src图像，对大于threshold的图像点，将max_value赋给dst图像，否则保持src图像原值 
// 日期：2014.03.28
//================================================================// 
int threshold(const IplImage* src,IplImage* dst,const double threshold,const unsigned char max_value)
{
	for(int r=0;r<src->height;r++)
	{
		unsigned char* psrc = (unsigned char*)src->imageData + r*src->widthStep;
		unsigned char* pdst = (unsigned char*)dst->imageData + r*dst->widthStep;
		for(int c=0;c<src->width;c++)
		{
			if(psrc[c] > threshold)
				pdst[c] = max_value;
			else
				pdst[c] = psrc[c];
		}
	}
	return 0;
}

//step2:车道轮廓提取
//sobel变换
//================================================================// 
// 功能： 对roi进行sobel变换，检测边缘后保存至roi_sobel
// 参数： 
// （入口） src: 源图像
// （出口） roi_sobel: 目标图像
// 返回： 函数正确返回0
// 主要思路：
//	temp取16位浮点型，否则出错，cvSobel后取绝对值，最后大津法阈值
//===============================================================//
int sobel(const IplImage* roi,IplImage* roi_sobel)
{
	IplImage* temp = cvCreateImage(cvGetSize(roi),IPL_DEPTH_16S,1);
	cvSobel(roi,temp,0,1,3);
	cvConvertScaleAbs(temp,roi_sobel,1,0);
	cvThreshold(roi_sobel,roi_sobel,0,255,CV_THRESH_OTSU);
	cvReleaseImage(&temp);
	return 0;
}

//================================================================// 
// 功能： 白线检测
// 参数： 
// （入口） roi: 源图像
//	    row: 检测第几行
//	    cb : 起始检测点 c(column) b(begin)
//	    ce : 终止检测点 c(column) e(end)
// 返回： 返回白点比率
// 主要思路：
//	  从cb开始检测，右亮度大于250则标记为cbegin,然后在cbegin
//        之后大于250的点，让cnt自加，最后用cnt/(ce-cb)算出白点率	
// 日期： 2014.03.28
//================================================================// 
float whitePointsRate(const IplImage* roi_sobel,const int row,const int cb,const int ce)
{
	unsigned char* ptr = (unsigned char*)roi_sobel->imageData+row*roi_sobel->widthStep;
	int cbegin;
	for(int c=cb;c<ce;c++)
	{
		if(ptr[c]>250)
		{
			cbegin = c;
			break;
		}
	}

	if(cbegin<0)
	{
		return 0;
	}

	int cnt = 0;
	int gap = 5;
	float rate;
	for(int c=cbegin;c<ce;c++)
	{
		if(ptr[c]>250)
		{
			cnt++;
			gap = 5;
		}
		else if(cnt>0 && gap>0)
			gap--;
	}
	rate = cnt/(float)(ce-cb);
	return rate;
}
//后续函数没有贴进来因为是想车道线检出后再加上去
///////////////////////////////////////////////////////////////////////////////////



int main(int argc,char** argv)
{
	//定义宏，当前在测试算法暂用car.mov这个文件替代摄像头
	#ifdef USE_CAMERA 
		CvCapture* input_video = cvCaptureFromCAM(0);
	#else 
		CvCapture* input_video = cvCreateFileCapture("/home/max/Documents/SRP/car.mov");
	#endif   

	CvSize video_size;
	video_size.height = (int)cvGetCaptureProperty(input_video,CV_CAP_PROP_FRAME_HEIGHT);//取得视频的高
	video_size.width = (int)cvGetCaptureProperty(input_video,CV_CAP_PROP_FRAME_WIDTH);//取得视频的宽，这样就定义好了video_size这个结构体，car.mov这个视频宽高度为848*480
	double fps = cvGetCaptureProperty(input_video,CV_CAP_PROP_FPS);
	int vfps = 1000 / fps; //计算每帧播放的时间

	//定义roi区域的大小(Region of interest),即是需处理的图像区域
	CvSize roi_size;
	roi_size.height = 0.35*video_size.height;//roi区域的高度，取下部分，除去了天空等无关的图像内容
	roi_size.width = video_size.width;//宽度保持不变，结果是截取了我们感兴趣的车道部分图像
	CvRect roi_rect = cvRect(0,0.65*video_size.height,roi_size.width,roi_size.height);

	IplImage* frame = NULL;
	IplImage* grey = cvCreateImage(video_size,IPL_DEPTH_8U,1);
	IplImage* roi = cvCreateImage(roi_size,IPL_DEPTH_8U,1);
	cvNamedWindow("orignal");
	cvNamedWindow("grey");
	cvNamedWindow("roi");
	//cvNamedWindow("roi2");
	unsigned int keyPress; 

	IplConvKernel* erode = cvCreateStructuringElementEx(10,1,0,0,CV_SHAPE_RECT,0);
	IplConvKernel* structure = cvCreateStructuringElementEx(5,1,3,0,CV_SHAPE_RECT,0);

	while(keyPress != 27) //while循环直到按下Esc键退出 
	{
		//载入下一帧图像
		frame = cvQueryFrame(input_video);

		//若视频加载完则返回-1退出
		if(frame == NULL)
		{
			fprintf(stderr,"Error: null frame received\n");
			return -1;
		}

		//将RGB图像帧转为灰度
		cvCvtColor(frame,grey,CV_RGB2GRAY);
		//剪取grey中roi_rect到roi
		crop(grey,roi,roi_rect);

		//两次阈值分割，结果更佳
		threshold(roi,roi,shadowBound(roi),255);
		threshold(roi,roi,shadowBound2(roi),255);
		
		//进一步提取出车身下的阴影 sobel-->腐蚀-->形态学变换
		sobel(roi,roi);

		cvErode(roi,roi,erode,1);

		cvMorphologyEx(roi,roi,0,structure,CV_MOP_CLOSE,1);

		//显示视频
		cvShowImage("orignal",frame);
		//cvShowImage("grey",grey);
		cvShowImage("roi",roi);

		//按空格键暂停(调试用)
		#ifdef debug
		keyPress = cvWaitKey(vfps);
		if (keyPress == 32)
				cvWaitKey(0);
		#endif 
	}

	//收尾工作要作好
	cvReleaseImage(&grey);
	cvReleaseImage(&roi);
	cvReleaseImage(&frame);

}
