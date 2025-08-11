/*
 Navicat Premium Data Transfer

 Source Server         :  xm-foods
 Source Server Type    : MySQL
 Source Server Version : 50744
 Source Host           : localhost:3306
 Source Schema         : yolo

 Target Server Type    : MySQL
 Target Server Version : 50744
 File Encoding         : 65001

 Date: 11/08/2025 00:26:00
*/

SET NAMES utf8mb4;
SET FOREIGN_KEY_CHECKS = 0;

-- ----------------------------
-- Table structure for admin
-- ----------------------------
DROP TABLE IF EXISTS `admin`;
CREATE TABLE `admin`  (
  `id` int(11) NOT NULL AUTO_INCREMENT COMMENT 'ID',
  `username` varchar(255) CHARACTER SET utf8mb4 COLLATE utf8mb4_unicode_ci NULL DEFAULT NULL COMMENT '用户名',
  `password` varchar(255) CHARACTER SET utf8mb4 COLLATE utf8mb4_unicode_ci NULL DEFAULT NULL COMMENT '密码',
  `name` varchar(255) CHARACTER SET utf8mb4 COLLATE utf8mb4_unicode_ci NULL DEFAULT NULL COMMENT '姓名',
  `avatar` varchar(255) CHARACTER SET utf8mb4 COLLATE utf8mb4_unicode_ci NULL DEFAULT NULL COMMENT '头像',
  `role` varchar(255) CHARACTER SET utf8mb4 COLLATE utf8mb4_unicode_ci NULL DEFAULT NULL COMMENT '角色标识',
  `phone` varchar(255) CHARACTER SET utf8mb4 COLLATE utf8mb4_unicode_ci NULL DEFAULT NULL COMMENT '电话',
  `email` varchar(255) CHARACTER SET utf8mb4 COLLATE utf8mb4_unicode_ci NULL DEFAULT NULL COMMENT '邮箱',
  PRIMARY KEY (`id`) USING BTREE
) ENGINE = InnoDB AUTO_INCREMENT = 3 CHARACTER SET = utf8mb4 COLLATE = utf8mb4_unicode_ci COMMENT = '管理员' ROW_FORMAT = DYNAMIC;

-- ----------------------------
-- Records of admin
-- ----------------------------
INSERT INTO `admin` VALUES (2, '555', '2333', '2', '3', 'ADMIN', '222', '12345');

-- ----------------------------
-- Table structure for rtsd
-- ----------------------------
DROP TABLE IF EXISTS `rtsd`;
CREATE TABLE `rtsd`  (
  `id` int(11) NOT NULL AUTO_INCREMENT COMMENT '传感器数据组ID',
  `time` varchar(255) CHARACTER SET utf8mb4 COLLATE utf8mb4_unicode_ci NULL DEFAULT NULL COMMENT '传感器获取时间',
  `t` double(11, 2) NULL DEFAULT NULL COMMENT '温度',
  `h` double(11, 2) NULL DEFAULT NULL COMMENT '湿度',
  `lng` double(11, 2) NULL DEFAULT NULL COMMENT '经度',
  `lat` double(11, 2) NULL DEFAULT NULL COMMENT '纬度',
  `altitude` double(11, 2) NULL DEFAULT NULL COMMENT '海拔',
  `cogt` double(11, 2) NULL DEFAULT NULL COMMENT '真北航向角',
  `speed` double(11, 2) NULL DEFAULT NULL COMMENT '速度',
  `weight` int(1) NULL DEFAULT NULL COMMENT '饲料重量\r\n',
  `waterlevel` int(1) NULL DEFAULT NULL COMMENT '水槽水位',
  `motor` int(1) NULL DEFAULT NULL COMMENT '割草机电机',
  `pump` int(1) NULL DEFAULT NULL COMMENT '水槽水泵',
  `light` int(1) NULL DEFAULT NULL COMMENT '光照',
  `door` int(1) NULL DEFAULT NULL COMMENT '圈门',
  `heat` int(1) NULL DEFAULT NULL COMMENT '加热',
  `fan` int(1) NULL DEFAULT NULL COMMENT '风机',
  PRIMARY KEY (`id`) USING BTREE
) ENGINE = InnoDB AUTO_INCREMENT = 15 CHARACTER SET = utf8mb4 COLLATE = utf8mb4_unicode_ci ROW_FORMAT = DYNAMIC;

-- ----------------------------
-- Records of rtsd
-- ----------------------------
INSERT INTO `rtsd` VALUES (2, '2', NULL, NULL, 30.00, 50.00, 100.00, 50.00, 30.00, 1, 1, 1, 0, 1, 1, 1, 1);
INSERT INTO `rtsd` VALUES (3, '2', NULL, NULL, 30.00, 50.00, 100.00, 50.00, 30.00, 1, 1, 1, 0, 1, 1, 1, 1);
INSERT INTO `rtsd` VALUES (14, '4', 30.50, 99.00, 40.00, 40.00, 100.00, 50.00, 30.00, 1, 0, 1, 0, 1, 1, 2, 1);

SET FOREIGN_KEY_CHECKS = 1;
