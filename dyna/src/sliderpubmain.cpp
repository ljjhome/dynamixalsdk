#include "sliderpub.h"
#include <QApplication>

int main(int argc, char** argv){
    
    QApplication app(argc, argv);
    ros::init(argc, argv,"speed_publish_node");
    ros::NodeHandle nodehandle;

    Slider window(nodehandle);
    window.show();
    return app.exec();
}
