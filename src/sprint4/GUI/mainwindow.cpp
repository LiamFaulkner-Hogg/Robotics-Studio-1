#include "mainwindow.h"
#include "./ui_mainwindow.h"


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    QPixmap logo("C:/Users/yvesg/OneDrive/Pictures/logo.png");
    ui->logo->setPixmap(logo.scaled(150,150,Qt::KeepAspectRatio));
}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::on_radioButton_4_clicked()
{
     QPixmap whouse("C:/Users/yvesg/OneDrive/Pictures/warehouse.png");
     ui->Image_label->setPixmap(whouse);
}


void MainWindow::on_radioButton_5_clicked()
{
    QPixmap house("C:/Users/yvesg/OneDrive/Pictures/house.png");
    ui->Image_label->setPixmap(house);
}



void MainWindow::on_radioButton_6_clicked()
{
    QPixmap out("C:/Users/yvesg/OneDrive/Pictures/outdoor.png");
    ui->Image_label->setPixmap(out);
}



void MainWindow::on_Spt_rob_button_clicked()
{
    QPixmap spot("C:/Users/yvesg/OneDrive/Pictures/spot.png");
    ui->model_label->setPixmap(spot.scaled(400,400,Qt::KeepAspectRatio));
}


void MainWindow::on_waffle_pi_button_clicked()
{
    QPixmap waff("C:/Users/yvesg/OneDrive/Pictures/waffpi.png");
    ui->model_label->setPixmap(waff.scaled(400,400,Qt::KeepAspectRatio));
}



void MainWindow::on_Burger_button_clicked()
{
    QPixmap burg("C:/Users/yvesg/OneDrive/Pictures/burger.png");
    ui->model_label->setPixmap(burg.scaled(400,400,Qt::KeepAspectRatio));
}



void MainWindow::on_pushButton_clicked()
{
    QMessageBox::StandardButton reply =
        QMessageBox::question(this,"Confirm", "Do you want to start the simulation?",
                              QMessageBox::Yes | QMessageBox::No);

    if(reply == QMessageBox::Yes)
    {
        qDebug() << "Yes is clicked";
    }
    else
    {
        qDebug() << "No is clicked";
    }
}

