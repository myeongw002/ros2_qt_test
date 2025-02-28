#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QThread>
#include <iostream>  // ✅ Debugging Output

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow) {
    ui->setupUi(this);

    // ROS2 Node Creation
    node = rclcpp::Node::make_shared("qt_gui_node");
    
    // ROS2 Publisher
    publisher = node->create_publisher<std_msgs::msg::String>("/chatter", 10);

    // ROS2 Subscriber
    subscriber = node->create_subscription<std_msgs::msg::String>(
        "/chatter_sub", 10, 
        std::bind(&MainWindow::chatterCallback, this, std::placeholders::_1)
    );

    // Button Click Event
    connect(ui->pushButton, &QPushButton::clicked, this, &MainWindow::onButtonClicked);

    // ✅ Start ROS2 processing in a separate thread
    rosThread = QThread::create([this]() { runRosNode(); });
    rosThread->start();
}

MainWindow::~MainWindow() {
    delete ui;
    rosThread->quit();  // ✅ Stop the ROS2 thread safely
    rosThread->wait();
    delete rosThread;
}

// ✅ Run ROS2 Executor in a Separate Thread
void MainWindow::runRosNode() {
    rclcpp::spin(node);  // Keep processing ROS2 messages
}

// Button Click - Publishes a ROS2 Message
void MainWindow::onButtonClicked() {
    auto message = std_msgs::msg::String();
    message.data = "Hello from Qt!";
    publisher->publish(message);
    RCLCPP_INFO(node->get_logger(), "Published: %s", message.data.c_str());
    std::cout << "Published: " << message.data << std::endl;  // ✅ Debugging Output
}

// ROS2 Subscriber Callback - Updates QLabel
void MainWindow::chatterCallback(const std_msgs::msg::String::SharedPtr msg) {
    QString qt_message = QString::fromStdString(msg->data);
    
    // ✅ Debugging Output
    std::cout << "Received: " << msg->data << std::endl;
    RCLCPP_INFO(node->get_logger(), "Received: %s", msg->data.c_str());

    // ✅ Update QLabel Safely in the Main Thread
    QMetaObject::invokeMethod(this, "updateLabel", Qt::QueuedConnection, Q_ARG(QString, qt_message));
}

// ✅ Update QLabel (Main Thread Safe)
void MainWindow::updateLabel(const QString &msg) {
    ui->label_4->setText(msg);  // Update QLabel text

    // ✅ Adjust label size dynamically based on content
    ui->label_4->adjustSize();
}

