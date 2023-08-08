

在 `MultiSensorEstimating::processEstimator()` 函数中如果没有手动设置初始坐标，可以根据 GNSS 原始观测值求出 SPP 解作为初始坐标。代码如下：

```C++
 else if (measurement.gnss && 
          estimatorTypeContains(SensorType::GNSS, type_)) {
   if (!spp_estimator_->addMeasurement(measurement)) {   // addMeasurement
     return false;
   }
   if (!spp_estimator_->estimate()) {    // estimate
     return false;
   }
   position_ecef = spp_estimator_->getPositionEstimate();  // getPositionEstimate
   measurement.gnss->position = position_ecef;
 }
```

主要是两个函数：`spp_estimator::addMeasurement`、`spp_estimator::getPositionEstimate`，下面会详细介绍。

