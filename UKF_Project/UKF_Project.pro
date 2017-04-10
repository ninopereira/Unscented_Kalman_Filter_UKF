#-------------------------------------------------
#
# Project created by QtCreator 2017-04-10T22:37:17
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = UKF_Project
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app


SOURCES += main.cpp \
    tools.cpp \
    ukf.cpp

HEADERS += \
    ground_truth_package.h \
    measurement_package.h \
    tools.h \
    ukf.h
