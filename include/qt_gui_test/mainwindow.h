#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QLabel>
#include <QThread>  // ✅ Add for ROS2 threading
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void onButtonClicked();   // Button click event to publish ROS2 message
    void updateLabel(const QString &msg);  // Function to update QLabel text

private:
    Ui::MainWindow *ui;
    rclcpp::Node::SharedPtr node;  // ROS2 Node
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;  // ROS2 Publisher
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber;  // ROS2 Subscriber
    QThread *rosThread;  // ✅ ROS2 Thread

    void chatterCallback(const std_msgs::msg::String::SharedPtr msg);  // ROS2 Callback Function
    void runRosNode();  // ✅ Function to run ROS2 in a separate thread
};

#endif // MAINWINDOW_H
