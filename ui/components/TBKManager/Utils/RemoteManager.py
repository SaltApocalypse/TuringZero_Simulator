from abc import ABC, abstractmethod
from typing import Any, Dict, List, Optional


class RemoteManager(ABC):
    """
    抽象基类：网络资源管理器
    定义了管理网络资源的标准接口
    """

    @abstractmethod
    def set(self, key: str, value: Any, **kwargs) -> bool:
        """
        设置/创建网络资源

        :param key: 资源唯一标识符
        :param value: 资源内容
        :param kwargs: 额外的配置参数
        :return: 操作是否成功
        """
        pass

    @abstractmethod
    def get(self, key: str, **kwargs) -> Optional[Any]:
        """
        获取网络资源

        :param key: 资源唯一标识符
        :param kwargs: 额外的查询参数
        :return: 资源内容，如果不存在返回None
        """
        pass

    @abstractmethod
    def put(self, key: str, value: Any, **kwargs) -> bool:
        """
        更新网络资源

        :param key: 资源唯一标识符
        :param value: 新的资源内容
        :param kwargs: 额外的更新参数
        :return: 操作是否成功
        """
        pass

    @abstractmethod
    def delete(self, key: str, **kwargs) -> bool:
        """
        删除网络资源

        :param key: 资源唯一标识符
        :param kwargs: 额外的删除参数
        :return: 操作是否成功
        """
        pass

    @abstractmethod
    def list(self, **kwargs) -> List[Any]:
        """
        列出网络资源

        :param filter_params: 过滤参数
        :param kwargs: 额外的列表查询参数
        :return: 资源列表
        """
        pass

    # @abstractmethod
    # def exists(self, key: str, **kwargs) -> bool:
    #     """
    #     检查资源是否存在
    #
    #     :param key: 资源唯一标识符
    #     :param kwargs: 额外的检查参数
    #     :return: 资源是否存在
    #     """
    #     pass
