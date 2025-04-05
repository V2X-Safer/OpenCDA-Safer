import datetime
import logging
import os
import opt

if not os.environ.get('LOG_DIR'):
    timestamp = datetime.datetime.now().strftime("%Y_%m_%d-%H_%M")
    os.environ['timestamp'] = timestamp
    os.environ['LOG_DIR'] = os.path.join(opt.param_dir, timestamp)
    os.makedirs(os.environ['LOG_DIR'], exist_ok=True)

timestamp = os.environ['timestamp']
LOG_DIR = os.environ['LOG_DIR']

# 主日志文件路径
MAIN_LOG_FILE = os.path.join(LOG_DIR, 'main.log')

# 控制台日志级别（新增）
CONSOLE_LOG_LEVEL = logging.INFO  # 默认INFO级别

# 初始化logger
logger = logging.getLogger('simlog')
logger.setLevel(logging.DEBUG)  # 设置日志级别为DEBUG

# 日志格式
main_formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
short_formatter = logging.Formatter('%(asctime)s - %(levelname)s  - %(message)s', datefmt='%H:%M:%S')
exception_formatter = logging.Formatter('%(asctime)s - %(levelname)s - \n%(exc_info)s', datefmt='%H:%M:%S')

# 主日志处理器（追加模式）
main_handler = logging.FileHandler(MAIN_LOG_FILE, mode='a') 
main_handler.setFormatter(main_formatter)
main_handler.setLevel(logging.DEBUG)

# 错误调用栈处理器（追加于主日志）
exception_handler = logging.FileHandler(MAIN_LOG_FILE, mode='a') 
exception_handler.setFormatter(exception_formatter)
exception_handler.setLevel(logging.ERROR)

# 控制台信息打印处理器
console_handler = logging.StreamHandler()
console_handler.setFormatter(short_formatter)
console_handler.setLevel(CONSOLE_LOG_LEVEL)  # 使用全局变量

# 添加处理器
logger.addHandler(main_handler)
logger.addHandler(console_handler)


def set_console_log_level(level):
    """设置控制台日志输出级别
    参数:
        level: 日志级别，如 logging.DEBUG, logging.INFO, logging.WARNING 等
    """
    global CONSOLE_LOG_LEVEL
    CONSOLE_LOG_LEVEL = level
    # logger.info(f"控制台日志级别已设置为: {level}")

def log_process_info(msg):
    """在主日志文件中记录信息"""
    logger.info(msg)
    
def log_process_debug(msg):
    """在主日志文件中记录信息"""
    logger.debug(msg)

def log_process_critical(msg):
    """记录关键错误信息并输出到控制台"""
    logger.critical(msg)

def log_event_info(process_name, msg):
    """根据进程名称创建子目录并记录信息"""
    
    # 创建子日志文件路径
    process_log_file = os.path.join(LOG_DIR, f'{process_name}.log')
    
    # 创建子日志处理器（追加模式）
    process_handler = logging.FileHandler(process_log_file, mode='a') 
    process_handler.setFormatter(main_formatter)
    process_handler.setLevel(logging.DEBUG)
    
    # 添加处理器并记录日志
    logger.addHandler(process_handler)
    logger.info(msg)
    
    # 移除处理器，避免重复添加
    logger.removeHandler(process_handler)
    

def log_event_critical(process_name, msg):
    """根据进程名称创建子目录并记录信息"""
    
    # 创建子日志文件路径
    process_log_file = os.path.join(LOG_DIR, f'{process_name}.log')
    
    # 创建子日志处理器（追加模式）
    process_handler = logging.FileHandler(process_log_file, mode='a')
    process_handler.setFormatter(main_formatter)
    process_handler.setLevel(logging.DEBUG)
    
    # 添加处理器并记录日志
    logger.addHandler(process_handler)
    logger.critical(msg)
    
    # 移除处理器，避免重复添加
    logger.removeHandler(process_handler)

def log_exception(msg, *args, exc_info=True, **kwargs):
    """
    记录异常信息和可选的追踪堆栈
    
    参数:
        msg: 错误消息
        exception: 异常对象，如果为None则仅记录消息
        traceback: 布尔值，指示是否记录完整的追踪堆栈
    """
    logger.error(msg, *args, exc_info=True, **kwargs)
    
if __name__ == '__main__':
    log_process_info('main log test')
    log_process_critical('main log test critical')
    log_process_debug('main test debug')
    log_event_info('test', 'event test')
    log_event_critical('test', 'event test critical')
    try:
        raise BaseException('test')
    except BaseException as e:
        log_exception("发生错误")
    
