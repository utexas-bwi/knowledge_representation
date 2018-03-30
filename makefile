
#Set below line to the path to your mysql connector c++ directory
MYSQL_CONCPP_DIR= /home/jason/mysql-connector



#After the second -I, list the path of this respository's include directory.
CPPFLAGS= -I $(MYSQL_CONCPP_DIR)/include -I /home/jason/hsr_ws/src/villa_krr/villa_kb/include -L $(MYSQL_CONCPP_DIR)/lib64

LDLIBS = -lmysqlcppconn8
CXXFLAGS = -std=c++11
high_level: interface.cpp high_level.cpp
