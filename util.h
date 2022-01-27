#ifndef UTIL_H
#define UTIL_H

#define TASK_STACK_SIZE (4096u)
#define	TASK_PRIORITY 	(5u)

#define LOG(str) printf(str "\n")
#define LOGF(str, args...) printf(str "\n", args)

#define CREATE_TASK_HANDLER(handler, task_handler_name) \
    static void task_handler_name() { \
        uint32_t ul_notification_value; \
        while (true) { \
            ul_notification_value = ulTaskNotifyTake(pdFALSE, portMAX_DELAY); \
            if (ul_notification_value == 1) { \
                handler(); \
            } \
        } \
    }

#endif /* UTIL_H */