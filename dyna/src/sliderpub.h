#include <QWidget>
#include <QSlider>

#include <QLabel>
#include <ros/ros.h>
class Slider:public QWidget{
    Q_OBJECT
public:
    Slider(ros::NodeHandle nh,QWidget *parent = 0);
private:
    QSlider *slider;
    QLabel *label;
    ros::NodeHandle nh_;
    ros::Publisher cmd_pub;
private slots:
    void updateLabelandNode(int value);
};
