#include "sliderpub.h"
#include <QHBoxLayout>
#include <std_msgs/Float64.h>
Slider::Slider(ros::NodeHandle nh, QWidget* parent):QWidget(parent), nh_(nh){
    
    QHBoxLayout *hbox = new QHBoxLayout(this);
    slider = new QSlider(Qt::Horizontal, this);
    slider->setRange(-1180,1180);
    label = new QLabel("0",this);

    hbox->addWidget(slider);
    hbox->addWidget(label);

    connect(slider,&QSlider::valueChanged, this, &Slider::updateLabelandNode);
    cmd_pub = nh_.advertise<std_msgs::Float64>("/dynamixel_speed",10);
}

void Slider::updateLabelandNode(int value){
    label->setNum(value/10.0);
    std_msgs::Float64 msg;
    msg.data = value/10.0;
    cmd_pub.publish(msg);
}
