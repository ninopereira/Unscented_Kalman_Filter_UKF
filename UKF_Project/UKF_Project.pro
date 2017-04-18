#-------------------------------------------------
#
# Project created by QtCreator 2017-04-18T16:17:02
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = UKF_Project
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app


SOURCES += \
    ../src/main.cpp \
    ../src/tools.cpp \
    ../src/ukf.cpp

HEADERS += \
    ../src/ground_truth_package.h \
    ../src/measurement_package.h \
    ../src/tools.h \
    ../src/ukf.h
