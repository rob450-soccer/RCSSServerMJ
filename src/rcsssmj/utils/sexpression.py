from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING, Any, Final, Union, cast, overload

from rcsssmj.utils.error import ParserError

if TYPE_CHECKING:
    from collections.abc import Iterator


OBC: Final[int] = ord('(')
"""The Open-Bracket-Character: '('"""


CBC: Final[int] = ord(')')
"""The Closing-Bracket-Character: ')'"""


SC: Final[int] = ord(' ')
"""The Space-Character: ' '"""


class MalformedSExpressionError(ParserError):
    """
    Error class representing a malformed symbolic expression / tree.
    """

    def __init__(self) -> None:
        """
        Construct a new malformed symbolic expression parser error.
        """

        super().__init__('Node not closed!')


class SExpression(Sequence[Union[bytes, bytearray, 'SExpression']]):
    """
    Representation of a symbolic expression.
    """

    def __init__(self) -> None:
        """
        Construct a new symbolic expression.
        """

        super().__init__()

        self._list: list[bytes | bytearray | SExpression] = []

    @staticmethod
    def from_array(data: bytes | bytearray) -> SExpression:
        """
        Parse the given UTF-8 encoded data array into a symbolic expression.
        """

        node = SExpression()

        SExpression._parse(node, data)

        return node

    def append_value(self, v: bytes | bytearray) -> bytes | bytearray:
        self._list.append(v)
        return v

    def append_node(self) -> SExpression:
        node = SExpression()
        self._list.append(node)
        return node

    def get_expr(self, i: int) -> SExpression:
        """
        Return the value at the given index as string.
        """

        return cast(SExpression, self._list[i])

    def get_str(self, i: int) -> str:
        """
        Return the value at the given index as string.
        """

        return cast(bytes, self._list[i]).decode()

    def get_int(self, i: int) -> int:
        """
        Return the value at the given index as int.
        """

        return int(cast(bytes, self._list[i]))

    def get_float(self, i: int) -> float:
        """
        Return the value at the given index as float.
        """

        return float(cast(bytes, self._list[i]))

    @staticmethod
    def _parse(node: SExpression, data: bytes | bytearray, start_idx: int = 0) -> int:
        """
        Internal recursive parser implementation.
        """

        idx: int = start_idx
        data_len = len(data)

        while idx < data_len:
            if data[idx] == OBC:
                # found a new sub expression
                if idx > start_idx:
                    node.append_value(data[start_idx:idx])

                start_idx = idx = SExpression._parse(node.append_node(), data, idx + 1)
            elif data[idx] == CBC:
                # found node terminator for the current expression
                if idx > start_idx:
                    node.append_value(data[start_idx:idx])

                return idx + 1
            elif data[idx] == SC:
                # found value terminator
                if idx > start_idx:
                    node.append_value(data[start_idx:idx])

                idx += 1
                start_idx = idx
            else:
                idx += 1

        if idx == data_len:
            if idx > start_idx:
                node.append_value(data[start_idx:])

            return idx + 1

        raise MalformedSExpressionError

    def __iter__(self) -> Iterator[bytes | bytearray | SExpression]:
        return self._list.__iter__()

    @overload
    def __getitem__(self, i: int) -> bytes | bytearray | SExpression: ...
    @overload
    def __getitem__(self, s: slice[Any, Any, Any]) -> Sequence[bytes | bytearray | SExpression]: ...
    def __getitem__(self, i: int | slice[Any, Any, Any]) -> bytes | bytearray | SExpression | Sequence[bytes | bytearray | SExpression]:
        return self._list[i]

    def __len__(self) -> int:
        return len(self._list)

    def __str__(self) -> str:
        return str(self._list)
