#pragma once

#if defined(_WIN32)
#if defined(KALMAN_IK_BUILD)
#define KALMAN_IK_API __declspec(dllexport)
#else
#define KALMAN_IK_API __declspec(dllimport)
#endif
#elif defined(__GNUC__)
#define KALMAN_IK_API __attribute__((visibility("default")))
#else
#define KALMAN_IK_API
#endif
