"""
AttributeWidgets' hierarchy is parallel to Attribute' hierarchy

An instance attr of Attribute can create its AttributeWidget counterPart
by calling attr.create_widget(name, parent).
"""

import numpy as np
from qtpy import QtCore, QtWidgets
import pyqtgraph as pg
from .spinbox import NumberSpinBox, IntSpinBox, FloatSpinBox, ComplexSpinBox
from .. import pyrpl_utils
from ..curvedb import CurveDB

import os
import sys

# TODO: try to remove widget_name from here (again)
class BaseAttributeWidget(QtWidgets.QWidget):
    """
    Base class for attribute widgets.

    The widget usually contains a label and a subwidget (property 'widget'
    of the instance), corresponding to the associated attribute. The subwidget
    is the created by the function make_widget.

    AttributeWidgets are always contained in a ModuleWidget and should be
    fully managed by this ModuleWidget.

    If widget_name=="", then only the subwidget is shown without label.

    A minimum widget should implmenet set_widget, _update,
    and possibly module_value.
    """
    value_changed = QtCore.Signal()

    def __init__(self, module, attribute_name, widget_name=None):
        super(BaseAttributeWidget, self).__init__()
        self.module = module
        self.attribute_name = attribute_name
        if widget_name is None:
            self.widget_name = self.attribute_name
        else:
            self.widget_name = widget_name
        self.setToolTip(self.attribute_descriptor.__doc__)
        self.layout_v = QtWidgets.QVBoxLayout()
        self.layout = self.layout_v
        if self.widget_name != "":
            self.label = QtWidgets.QLabel(self.widget_name)
            self.layout.addWidget(self.label, 0) # stretch=0
            self.layout.addStretch(1)
        self.layout_v.setContentsMargins(0, 0, 0, 0)
        self._make_widget()
        self.layout.addWidget(self.widget, 0) # stretch=0
        self.layout.addStretch(1)
        self.setLayout(self.layout)
        self.write_attribute_value_to_widget()
        # this is very nice for debugging, but should probably be removed later
        setattr(self.module, '_'+self.attribute_name+'_widget', self)

    @property
    def attribute_descriptor(self):
        return getattr(self.module.__class__, self.attribute_name)

    @property
    def attribute_value(self):
        return getattr(self.module, self.attribute_name)

    @attribute_value.setter
    def attribute_value(self, v):
        setattr(self.module, self.attribute_name, v)

    @property
    def widget_value(self):
        """ Property for the current value of the widget.

        The associated setter takes care of not re-emitting signals when the
        gui value is modified through the setter. """
        return self._get_widget_value()

    @widget_value.setter
    def widget_value(self, v):
        if not self.widget.hasFocus():
            try:
                self.widget.blockSignals(True)
                self._set_widget_value(v)
            finally:
                self.widget.blockSignals(False)

    def write_widget_value_to_attribute(self):
        self.attribute_value = self.widget_value
        # since there is no protection there, the value will propagate
        #    widget manipulation --> module change --> widget change
        # However, since there is a blockSignal in the setter of widget_value,
        # the loop stops there. However a widget manipulation (a click on the
        # widget arrow for instance) results in 2 widgets overwrites

        # it does not hurt to imitate the signal of the subwidget,
        # even though most of the time nothing is connected to it
        self.value_changed.emit()

    def write_attribute_value_to_widget(self):
        """ trivial helper function, updates widget value from the attribute"""
        current_value = self.attribute_value
        if current_value is None:
            #  SelectAttributes might have a None value
            self.module._logger.warning("Cannot set widget %s of attribute "
                                        "%s.%s to the current value "
                                        "'None'.",
                                        self.widget_name,
                                        self.module.name,
                                        self.attribute_name)
        else:
            self.widget_value = current_value

    def editing(self):
        """
        User is editing the property graphically don't mess up with him
        :return:
        """
        return self.widget.editing()

    def set_horizontal(self):
        """ puts the label to the left of the widget instead of atop """
        self.layout_h = QtWidgets.QHBoxLayout()
        if hasattr(self, 'label'):
            self.layout_v.removeWidget(self.label)
            self.layout_h.addWidget(self.label)
            self.layout.addStretch(1)
        self.layout_v.removeWidget(self.widget)
        self.layout_h.addWidget(self.widget)
        self.layout.addStretch(1)
        self.layout_v.addLayout(self.layout_h)
        self.layout = self.layout_h

    def _make_widget(self):
        """
        create the new widget.

        Overwrite in derived class.
        """
        self.widget = None

    def _get_widget_value(self):
        """
        returns the current value shown by the widget.

        The type that is returned is understood by the underlying attribute.

        Overwrite in derived class.
        """
        return self.widget.value()

    def _set_widget_value(self, new_value):
        """
        Changes the value displayed in the widget.

        Overwrite in derived class.
        """
        self.widget.setValue(new_value)

    def wheelEvent(self, event):
        """
        Handle mouse wheel event.
        """
        # We generally disable the mouse wheel because it is a
        # frequent source of errors / undesired changes of variables
        # Here, it is simply forwarded upwards in hierarchy.
        return super(BaseAttributeWidget, self).wheelEvent(event)


class StringAttributeWidget(BaseAttributeWidget):
    """
    Widget for string values.

    Commits on Enter or focus-out, NOT per keystroke. The box used to be wired
    to textChanged, so typing "20,20" wrote the attribute five times — "2",
    "20", "20,", "20,2", "20,20" — and every one of those was a real setter
    call: a config save, and for setters that act on write (a reflash, a
    device command, a re-parse) a real action on a half-typed value. Worse,
    the intermediate strings are usually INVALID, so parsers saw garbage and
    logged for every character.

    editingFinished is the commit point, guarded by _editing so a bare
    focus-out cannot re-commit a value nobody touched, nor revert an external
    update that landed while the box had focus (same idiom as
    FileAttributeWidget below).
    """
    def _make_widget(self):
        # True once the user has TYPED since the last programmatic set
        self._editing = False
        self.widget = QtWidgets.QLineEdit()
        self.widget.setMaximumWidth(200)
        # textEdited fires ONLY on user typing, never on programmatic setText
        self.widget.textEdited.connect(self._on_text_edited)
        self.widget.editingFinished.connect(self._on_edit_finished)

    def _on_text_edited(self, _text):
        self._editing = True

    def _on_edit_finished(self):
        """Enter or focus-out. editingFinished fires on EVERY focus-out, so
        commit only when the text was actually edited."""
        if not self._editing:
            return
        self._editing = False
        self.write_widget_value_to_attribute()

    def editing(self):
        """An uncommitted edit is in the box — callers use this to hold off
        overwriting it with an external value (BaseAttributeWidget.editing
        delegates to the subwidget, which for a plain QLineEdit has no such
        method, so this override is what makes the contract work here)."""
        return self._editing

    def _get_widget_value(self):
        return str(self.widget.text())

    def _set_widget_value(self, new_value):
        # an authoritative value from the attribute: drop any in-progress edit
        # so the box can never keep showing stale half-typed text
        self._editing = False
        self.widget.setText(new_value)


class FileAttributeWidget(BaseAttributeWidget):
    """
    Widget for file entry with browse button.

    The full path is the actual value, but to keep the box compact only the
    file NAME is shown when the box is idle (the complete path is in the box's
    tooltip); the full path is revealed for editing while the box has focus.
    The browse dialog offers the attribute's file-type filter (if any) plus an
    always-present "All files" entry.
    """
    # last directory a file was picked from, shared by ALL file attributes for
    # the session (class attribute; deliberately not persisted as a setting)
    _last_dir = ''

    def _make_widget(self):
        # the complete path (the box only DISPLAYS the basename when idle)
        self._full_path = ''
        # True once the user has TYPED into the box since the last programmatic
        # set — distinguishes a real edit from a plain focus-out. Without it,
        # editingFinished (which also fires on every focus-out) wrote the
        # displayed text back, so an external value change while the box had
        # focus was silently reverted and the display de-synced from the value.
        self._editing = False
        self.widget = QtWidgets.QWidget()
        layout = self.lay = QtWidgets.QHBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        self.widget.setLayout(layout)
        self.lineedit = QtWidgets.QLineEdit()
        layout.addWidget(self.lineedit)
        button = QtWidgets.QPushButton('...')
        button.setMaximumWidth(20)
        button.clicked.connect(self._on_browse)
        layout.addWidget(button, 0)
        # textEdited fires ONLY on user typing (not on programmatic setText);
        # editingFinished commits on Enter / focus-out (not per keystroke: the
        # value is a full path and some setters act on write, e.g. reflash).
        self.lineedit.textEdited.connect(self._on_text_edited)
        self.lineedit.editingFinished.connect(self._on_edit_finished)
        # reveal the full path on focus-in so the whole path stays editable
        self.lineedit.installEventFilter(self)

    def _on_text_edited(self, _text):
        self._editing = True

    def _refresh_display(self):
        """Keep the box in sync with _full_path: tooltip is always the full
        path, the box shows the full path while focused (fully editable) and
        the basename when idle. Skipped only while the user has an uncommitted
        edit in progress, so an external value change always re-syncs."""
        self.lineedit.setToolTip(self._full_path)
        if self._editing:
            return
        shown = self._full_path if self.lineedit.hasFocus() \
            else (os.path.basename(self._full_path) or self._full_path)
        if str(self.lineedit.text()) != shown:
            self.lineedit.setText(shown)

    def eventFilter(self, obj, event):
        # on focus-in, reveal the full path (not just the basename) for editing
        if obj is self.lineedit and event.type() == QtCore.QEvent.FocusIn:
            self._refresh_display()
        return super().eventFilter(obj, event)

    def _on_edit_finished(self):
        # editingFinished also fires on a plain focus-out — commit ONLY a real
        # user edit, else the revealed full path would be written back over an
        # external update. Either way, re-sync the display to the actual value.
        if self._editing:
            self._editing = False
            text = str(self.lineedit.text()).strip()
            # a bare file name (no directory typed) is resolved against the
            # directory of the current value — the box shows only the basename,
            # so editing it to a sibling file keeps it in the same folder
            if text and not os.path.dirname(text):
                base_dir = os.path.dirname(self._full_path)
                if base_dir:
                    text = os.path.join(base_dir, text)
            if text != self._full_path:
                self._full_path = text
                self.write_widget_value_to_attribute()
        self._refresh_display()

    def _dialog_filter(self):
        """Build the QFileDialog filter string from the attribute's optional
        ``file_filter`` (a Qt filter string or a list of them), always adding
        an all-files entry so nothing is hidden."""
        filters = getattr(self.attribute_descriptor, 'file_filter', None)
        if not filters:
            parts = []
        elif isinstance(filters, str):
            parts = [filters]
        else:
            parts = list(filters)
        if not any(('(*)' in p) or ('(*.*)' in p) for p in parts):
            parts.append('All files (*)')
        return ';;'.join(parts)

    def _on_browse(self):
        # start at the session's last browsed directory; before any browse,
        # fall back to the directory of the attribute's current value
        start_dir = FileAttributeWidget._last_dir
        if not start_dir:
            current = self._full_path.strip()
            if current:
                d = current if os.path.isdir(current) \
                    else os.path.dirname(current)
                if os.path.isdir(d):
                    start_dir = d
        filename = QtWidgets.QFileDialog.getOpenFileName(
            self.widget, 'Select file', start_dir, self._dialog_filter())
        if filename and filename[0]:
            self._editing = False
            self._full_path = filename[0]
            self.write_widget_value_to_attribute()
            self._refresh_display()
            FileAttributeWidget._last_dir = os.path.dirname(filename[0])

    def _get_widget_value(self):
        return self._full_path

    def _set_widget_value(self, new_value):
        # an external/authoritative value: drop any in-progress edit and
        # re-sync the display so the box can never show a stale path
        self._editing = False
        self._full_path = new_value or ''
        self._refresh_display()


class TextAttributeWidget(StringAttributeWidget):
    """
    Property for multiline string values.

    Same commit-on-finish rule as StringAttributeWidget, and the same reason:
    a multiline value written per keystroke is never anything but half-typed.
    QTextEdit has no editingFinished, so the commit point is the focus-out,
    caught with an event filter. The "user edited" flag comes from
    textChanged, which unlike QLineEdit's textEdited also fires on
    PROGRAMMATIC changes -- so _set_widget_value clears the flag after
    writing rather than before. The base class happens to block signals
    around that call, but this must not depend on it.
    """
    def _make_widget(self):
        self._editing = False
        self.widget = QtWidgets.QTextEdit()
        self.widget.textChanged.connect(self._on_text_edited)
        self.widget.installEventFilter(self)

    def _on_text_edited(self, *_args):
        self._editing = True

    def eventFilter(self, obj, event):
        if obj is self.widget and event.type() == QtCore.QEvent.FocusOut:
            self._on_edit_finished()
        return super(TextAttributeWidget, self).eventFilter(obj, event)

    def _set_widget_value(self, new_value):
        # clear AFTER the write: the setText above re-raises textChanged when
        # signals are not blocked, which would leave a phantom edit pending
        # and commit it on the next focus-out
        super(TextAttributeWidget, self)._set_widget_value(new_value)
        self._editing = False

    def _get_widget_value(self):
        return str(self.widget.toPlainText())


class NumberAttributeWidget(BaseAttributeWidget):
    """
    Base widget for float and int.
    """
    SpinBox = NumberSpinBox

    def _make_widget(self):
        super(NumberAttributeWidget, self)._make_widget()
        self.widget = self.SpinBox(None)
        self.widget.value_changed.connect(self.write_widget_value_to_attribute)
        self.widget.setSingleStep(self.attribute_descriptor.increment)
        self.widget.setMaximum(self.attribute_descriptor.max)
        self.widget.setMinimum(self.attribute_descriptor.min)
        if self.attribute_descriptor.log_increment:
            self.widget.set_log_increment()

    def _get_widget_value(self):
        return self.widget.value()

    def editing(self):
        return self.widget.line.hasFocus()

    def set_per_second(self, val):
        self.widget.set_per_second(val)

    def set_log_increment(self):
        self.widget.set_log_increment()

    def keyPressEvent(self, event):
        """ forwards all key events to spinbox """
        return self.widget.keyPressEvent(event)

    def keyReleaseEvent(self, event):
        """ forwards all key events to spinbox """
        return self.widget.keyReleaseEvent(event)


class IntAttributeWidget(NumberAttributeWidget):
    """
    Widget for integer values.
    """
    SpinBox = IntSpinBox


class FloatAttributeWidget(NumberAttributeWidget):
    """
    Widget for float values
    """
    SpinBox = FloatSpinBox


class ComplexAttributeWidget(FloatAttributeWidget):
    """
    Widget for complex values
    """
    SpinBox = ComplexSpinBox


class ListElementWidget(BaseAttributeWidget):
    """
    this is a wrapper class to embed any AttributeWidget as an element of
    BasePropertyListPropertyWidget. Its usage is found in the property
    element_widget_cls of BasePropertyListPropertyWidget.
    """
    def __init__(self, parent, startindex, *args, **kwargs):
        self.parent = parent
        self.startindex = startindex
        super(ListElementWidget, self).__init__(*args, **kwargs)
        self.set_horizontal()
        self.button_remove = QtWidgets.QPushButton('-')
        self.button_remove.clicked.connect(self.remove_this_element)
        self.button_remove.setFixedWidth(2 * 10)
        self.layout.addWidget(self.button_remove, 0) # stretch=0
        self.layout.addStretch(1)
        # this is very nice for debugging, but should probably be removed later
        setattr(self.module, '_'+self.attribute_name+'_widget', self.parent)

    def remove_this_element(self):
        self.parent.attribute_value.__delitem__(index=self.index)

    @property
    def index(self):
        if self in self.parent.widgets:
            return self.parent.widgets.index(self)
        else:
            return self.startindex

    @property
    def attribute_value(self):
        return getattr(self.module, self.attribute_name)[self.index]

    @attribute_value.setter
    def attribute_value(self, v):
        getattr(self.module, self.attribute_name)[self.index] = v

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            # left button selects the item
            self.parent.attribute_value.selected = self.index
        elif event.button() == QtCore.Qt.RightButton:
            pass  # no functionality so far
        return super(ListElementWidget, self).mousePressEvent(event)

    def focusInEvent(self, QFocusEvent):
        self.parent.attribute_value.selected = self.index

    # keyboard interface
    # def keyPressEvent(self, event):
    #     """ forwards all key events to selected widget """
    #     if self.parent.selected is not None:
    #         return self.parent.selected.keyPressEvent(event)
    #     else:
    #         super(ListElementWidget, self).keyPressEvent(event)
    #
    #
    # def keyReleaseEvent(self, event):
    #     """ forwards all key events to selected widget """
    #     if self.parent.selected is not None:
    #         return self.parent.selected.keyReleaseEvent(event)
    #     else:
    #         super(ListElementWidget, self).keyReleaseEvent(event)


class BasePropertyListPropertyWidget(BaseAttributeWidget):
    """
    A widget for a list of Attributes, deriving its functionality from the
    underlying widgets
    """
    def _make_widget(self):
        self.widget = QtWidgets.QFrame()
        self.widget_layout = QtWidgets.QVBoxLayout()
        self.widget.setLayout(self.widget_layout)
        self.widgets = []
        self.button_add = QtWidgets.QPushButton("+")
        self.button_add.clicked.connect(self.append_default)
        self.widget_layout.addWidget(self.button_add)
        self.widget_layout.addStretch(1)
        self.widget_layout.setContentsMargins(0, 0, 0, 0)
        self.selected = None
        self.update_widget_names()

    @property
    def element_widget_cls(self):
        return type("ElementWidget",
                    (ListElementWidget,
                     self.attribute_descriptor.element_cls._widget_class, ),
                    {})

    def update_attribute_by_name(self, new_value_list):
        current_list, operation, index, value = new_value_list
        if operation == "insert":
            self.insert(index, value)
        elif operation == "setitem":
            self.setitem(index, value)
        elif operation == "delitem":
            self.delitem(index)
        elif operation == "select":
            self.select(index)
        else:
            self.module._logger.error("%s.%s_widget.update_attribute_by_name "
                                      "was called with wrong arguments: %s",
                                      self.module.name,
                                      self.name,
                                      new_value_list)

    def append_default(self):
        self.attribute_value.append()

    def insert(self, index, value):
        """"
        make a new element widget - this function is called by the attribute
        """
        element_widget = self.element_widget_cls(self,
                                                 index,
                                                 self.module,
                                                 self.attribute_name,
                                                 widget_name=str(index))
        self.widgets.insert(index, element_widget)
        if index+1 >= len(self.widgets):
            # widget must be inserted at the end
            insert_before = self.button_add
        else:
            # widget was inserted before another one
            insert_before = self.widgets[index+1]
        self.widget_layout.insertWidget(self.widget_layout.indexOf(insert_before),
                                        element_widget)
        self.update_widget_names()

    def setitem(self, index, value):
        self.widgets[index].widget_value = value

    def delitem(self, index=-1):
        if self.selected == index:
            self.selected = None
        widget = self.widgets.pop(index)
        widget.hide()
        self.widget_layout.removeWidget(widget)
        widget.deleteLater()
        self.update_widget_names()
        self.module._logger.debug('delitem concluded')

    def select(self, index):
        for i, widget in enumerate(self.widgets):
            if i == index:
                widget.setStyleSheet("background-color: #EAE1E1")
                widget.setFocus()
            else:
                widget.setStyleSheet("")

    def update_widget_names(self):
        for widget in self.widgets:
            if hasattr(widget, 'label'):
                widget.label.setText('('+str(widget.index)+')')

    def _get_widget_value(self):
        return [widget.widget_value for widget in self.widgets]

    def _set_widget_value(self, new_values):
        for index, new_value in enumerate(new_values):
            # replace or append new values
            try:
                self.setitem(index, new_value)
            except IndexError:
                self.insert(index, new_value)
            # remove the trailing items
            while len(self) > len(new_values):
                self.delitem()

    def editing(self):
        edit = False
        for widget in self.widgets:
            edit = edit or widget.editing()
        return edit

    @property
    def number(self):
        return self.__len__()

    def __len__(self):
        return len(self.widgets)


class ListComboBox(QtWidgets.QWidget):
    # exclusively used by FilterAttributeWidget, can be replaced by sth else
    # TODO: can be replaced by SelectAttributeWidget
    value_changed = QtCore.Signal()

    def __init__(self, number, name, options, decimals=3):
        super(ListComboBox, self).__init__()
        self.setToolTip("First order filter frequencies \n"
                        "negative values are for high-pass \n"
                        "positive for low pass")
        self.lay = QtWidgets.QHBoxLayout()
        self.lay.setContentsMargins(0, 0, 0, 0)
        self.combos = []
        self.options = options
        self.decimals = decimals
        for i in range(number):
            combo = QtWidgets.QComboBox()
            self.combos.append(combo)
            combo.addItems(self.options)
            combo.currentIndexChanged.connect(self.value_changed)
            self.lay.addWidget(combo)
        self.setLayout(self.lay)

    def change_options(self, new_options):
        self.options = new_options
        for combo in self.combos:
            combo.blockSignals(True)
            combo.clear()
            combo.addItems(new_options)
            combo.blockSignals(False)

    def get_list(self):
        return [float(combo.currentText()) for combo in self.combos]

    def set_max_cols(self, n_cols):
        """
        If more than n boxes are required, go to next line
        """
        if len(self.combos)<=n_cols:
            return
        for item in self.combos:
            self.lay.removeWidget(item)
        self.v_layouts = []
        n = len(self.combos)
        n_rows = int(np.ceil(n*1.0/n_cols))
        j = 0
        for i in range(n_cols):
            layout = QtWidgets.QVBoxLayout()
            self.lay.addLayout(layout)
            for j in range(n_rows):
                index = i*n_rows + j
                if index>=n:
                    break
                layout.addWidget(self.combos[index])

    def set_list(self, val):
        if not np.iterable(val):
            val = [val]
        for i, v in enumerate(val):
            #v = str(int(v))
            v = ('{:.' + str(self.decimals) + 'e}').format(float(v))
            index = self.options.index(v)
            self.combos[i].setCurrentIndex(index)


class FilterAttributeWidget(BaseAttributeWidget):
    """
    Property for list of floats (to be chosen in a list of valid_frequencies)
    The attribute descriptor needs to expose a function valid_frequencies(module)
    """
    decimals = 3

    def __init__(self, module, attribute_name, widget_name=None):
        val = getattr(module, attribute_name)
        if np.iterable(val):
            self.number = len(val)
        else:
            self.number = 1
        self.options = getattr(module.__class__, attribute_name).valid_frequencies(module)
        super(FilterAttributeWidget, self).__init__(module, attribute_name,
                                                    widget_name=widget_name)

    def _make_widget(self):
        """
        Sets up the widget (here a QDoubleSpinBox)
        :return:
        """
        self.widget = ListComboBox(self.number,
                                   "",
                                   self._format_options(),
                                   decimals=self.decimals)
        self.widget.value_changed.connect(self.write_widget_value_to_attribute)

    def _format_options(self):
        return [('{:.'+str(self.decimals)+'e}').format(
            float(option)) for option in self.options]

    def refresh_options(self, module):
        self.options = getattr(module.__class__,
                               self.attribute_name).valid_frequencies(module)
        self.widget.change_options(self._format_options())

    def _get_widget_value(self):
        return self.widget.get_list()

    def _set_widget_value(self, new_value):
        if isinstance(new_value, str) or not np.iterable(new_value):
            # only 1 element in the FilterAttribute, make a list
            # for consistency with other filters (this used to be basestring)
            new_value = [new_value]
        self.widget.set_list(new_value)

    def set_max_cols(self, n_cols):
        self.widget.set_max_cols(n_cols)


class SelectAttributeWidget(BaseAttributeWidget):
    """
    Multiple choice property.
    """
    def _make_widget(self):
        self.widget = QtWidgets.QComboBox()
        self.widget.addItems(self.options)
        self.widget.currentIndexChanged.connect(self.write_widget_value_to_attribute)

    @property
    def options(self):
        opt = self.attribute_descriptor.options(self.module).keys()
        opt = [str(v) for v in opt]
        if len(opt) == 0:
            opt = [""]
        return opt

    def _get_widget_value(self):
        return str(self.widget.currentText())
        #try:
        #    return str(self.widget.currentText())
        # except ValueError as e1:
        #     # typically string - int - conversion related
        #     options = self.options
        #     try:
        #         index = [str(k) for k in options].index(str(self.widget.currentText()))
        #     except ValueError as e2:
        #         raise e1
        #     else:
        #         setattr(self.module, self.attribute_name, options[index])

    def _set_widget_value(self, new_value):
        try:
            index = self.options.index(str(new_value))
        except (IndexError, ValueError):
            self.module._logger.warning("SelectWidget %s could not find "
                                        "current value %s in the options %s",
                                        self.attribute_name,
                                        new_value,
                                        self.options)
            index = 0
        self.widget.setCurrentIndex(index)

    def change_options(self, new_options=None):
        """
        The options of the combobox can be changed dynamically.

        new_options is an argument that is ignored, since the new options
        are available as a property to the widget already.
        """
        self.widget.blockSignals(True)
        #self.defaults = new_options
        self.widget.clear()
        #self.widget.addItems(new_options)
        # do not trust the new options, rather call options again
        self.widget.addItems(self.options)
        self.widget_value = self.attribute_value
        self.widget.blockSignals(False)


class LedAttributeWidget(BaseAttributeWidget):
    """ Boolean property with a button whose text and color indicates whether """
    def _make_widget(self):
        desc = pyrpl_utils.recursive_getattr(self.module, '__class__.' + self.attribute_name)
        val = pyrpl_utils.recursive_getattr(self.module, self.attribute_name)
        self.widget = QtWidgets.QPushButton("setting up...")
        self.widget.clicked.connect(self.button_clicked)

    def _set_widget_value(self, new_value):
        if new_value == True:
            color = 'green'
            text = 'stop'
        else:
            color = 'red'
            text = 'start'
        self.widget.setStyleSheet("background-color:%s"%color)
        self.widget.setText(text)

    def button_clicked(self):
        self.attribute_value = not self.attribute_value


class BoolAttributeWidget(BaseAttributeWidget):
    """
    Checkbox for boolean attributes
    """
    def _make_widget(self):
        self.widget = QtWidgets.QCheckBox()
        self.widget.stateChanged.connect(self.write_widget_value_to_attribute)

    def _get_widget_value(self):
        return (self.widget.checkState() == 2)

    def _set_widget_value(self, new_value):
        self.widget.setCheckState(new_value * 2)


class BoolIgnoreAttributeWidget(BoolAttributeWidget):
    """
    Like BoolAttributeWidget with additional option 'ignore' that is
    shown as a grey check in GUI
    """
    _gui_to_attribute_mapping = pyrpl_utils.Bijection({0: False,
                                                       1: 'ignore',
                                                       2: True})

    def _make_widget(self):
        """
        Sets the widget (here a QCheckbox)
        :return:
        """
        self.widget = QtWidgets.QCheckBox()
        self.widget.setTristate(True)
        self.widget.stateChanged.connect(self.write_widget_value_to_attribute)
        self.setToolTip("Checked:\t    on\nUnchecked: off\nGrey:\t    ignore")

    def _get_widget_value(self):
        return self._gui_to_attribute_mapping[self.widget.checkState()]

    def _set_widget_value(self, new_value):
        self.widget.setCheckState(
            self._gui_to_attribute_mapping.inverse[new_value])


class DataWidget(pg.GraphicsLayoutWidget):
    """
    A widget to plot real or complex datasets. To plot data, use the
    function _set_widget_value(new_value, transform_magnitude)

    new_value is a a tuple (x, y), with x the x values, y, a 1D array for a
    single curve or a 2D array for multiple curves. If at least one of the
    curve is complex, magnitude and phases will be plotted.

    transform_magnitude is the function to transform magnitude data.
    """
    _defaultcolors = ['m', 'b', 'g', 'r', 'y', 'c', 'o', 'w']
    def __init__(self, title=None):
        super(DataWidget, self).__init__(title=title)
        self.plot_item = self.addPlot(title="Curve")
        self.plot_item_phase = self.addPlot(row=1, col=0,
                                                   title="Phase (deg)")
        self.plot_item_phase.setXLink(self.plot_item)
        self.plot_item.showGrid(y=True, alpha=1.)
        self.plot_item_phase.showGrid(y=True, alpha=1.)
        self.curves = []  # self.plot_item.plot(pen='g')
        self.curves_phase = []  # self.plot_item_phase.plot(pen='g')
        self._is_real = True
        self._set_real(True)

    def _set_widget_value(self, new_value, transform_magnitude=lambda data :
    20. * np.log10(np.abs(data) + sys.float_info.epsilon)):
        if new_value is None:
            return
        x, y = new_value
        shape = np.shape(y)
        if len(shape) > 2:
            raise ValueError("Data cannot be larger than 2 "
                             "dimensional")
        if len(shape) == 1:
            y = [y]
        self._set_real(np.isreal(y).all())
        for i, values in enumerate(y):
            self._display_curve_index(x, values, i, transform_magnitude=transform_magnitude)
            self.curves[i].show()
        while (i + 1 < len(self.curves)):  # delete remaining curves
            i += 1
            self.curves[i].hide()

    def _display_curve_index(self, x, values, i, transform_magnitude):
        y_mag = transform_magnitude(values)
        y_phase = np.zeros(len(values)) if self._is_real else \
            self._phase(values)
        if len(self.curves) <= i:
            color = self._defaultcolors[i % len(self._defaultcolors)]
            self.curves.append(self.plot_item.plot(pen=color))
            self.curves_phase.append(self.plot_item_phase.plot(pen=color))
        self.curves[i].setData(x, y_mag)
        self.curves_phase[i].setData(x, y_phase)

    def _set_real(self, bool):
        self._is_real = bool
        if bool:
            self.plot_item_phase.hide()
            self.plot_item.setTitle("")
        else:
            self.plot_item_phase.show()
            self.plot_item.setTitle("Magnitude (dB)")

    def _phase(self, data):
        """ little helpers """
        return np.angle(data, deg=True)

    def setRange(self, *args, **kwds):
        self.plot_item.setRange(*args, **kwds)

    def autoRange(self):
        self.plot_item.autoRange()
        self.plot_item_phase.autoRange()


class PlotAttributeWidget(BaseAttributeWidget):
    _defaultcolors = ['g', 'r', 'b', 'y', 'c', 'm', 'o', 'w']

    def time(self):
        return pyrpl_utils.time()

    def _make_widget(self):
        """
        Sets the widget (here a QCheckbox)
        :return:
        """
        self.widget = pg.GraphicsLayoutWidget(title="Plot")
        legend = getattr(self.module.__class__, self.attribute_name).legend
        self.pw = self.widget.addPlot(title="%s vs. time (s)"%legend)
        self.plot_start_time = self.time()
        self.curves = {}
        setattr(self.module.__class__, '_' + self.attribute_name + '_pw', self.pw)

    def _set_widget_value(self, new_value):
        try:
            args, kwargs = new_value
        except:
            if isinstance(new_value, dict):
                args, kwargs = [], new_value
            elif isinstance(new_value, list):
                args, kwargs = new_value, {}
            else:
                args, kwargs = [new_value], {}
        for k in kwargs.keys():
            v = kwargs.pop(k)
            kwargs[k[0]] = v
        i=0
        for value in args:
            while self._defaultcolors[i] in kwargs:
                i += 1
            kwargs[self._defaultcolors[i]] = value
        t = self.time()-self.plot_start_time
        for color, value in kwargs.items():
            if value is not None:
                if not color in self.curves:
                    self.curves[color] = self.pw.plot(pen=color)
                curve = self.curves[color]
                x, y = curve.getData()
                if x is None or y is None:
                    x, y = np.array([t]), np.array([value])
                else:
                    x, y = np.append(x, t), np.append(y, value)
                curve.setData(x, y)

    def _magnitude(self, data):
        """ little helpers """
        return 20. * np.log10(np.abs(data) + sys.float_info.epsilon)

    def _phase(self, data):
        """ little helpers """
        return np.angle(data, deg=True)


class DataAttributeWidget(PlotAttributeWidget):
    """
    Plots a curve (complex or real), with an array as input.
    """

    def _make_widget(self):
        self.widget = pg.GraphicsLayoutWidget(title="Curve")
        self.plot_item = self.widget.addPlot(title="Curve")
        self.plot_item_phase = self.widget.addPlot(row=1, col=0, title="Phase (deg)")
        self.plot_item_phase.setXLink(self.plot_item)
        self.plot_item.showGrid(y=True, alpha=1.)
        self.plot_item_phase.showGrid(y=True, alpha=1.)
        self.curve = self.plot_item.plot(pen='g')
        self.curve_phase = self.plot_item_phase.plot(pen='g')
        self._is_real = True
        self._set_real(True)

    #def _set_widget_value(self, new_value):
    #   data = new_value
    #    if data is None:
    #        return
    #    shape = np.shape(new_value)
    #    if len(shape)>2:
    #        raise ValueError("Shape of data should be (1) or (2, 1)")
    #    if len(shape)==1:
    #        x = np.linspace(0, len(data), len(data))
    #        y = [data]
    #    if len(shape)==2:
    #        if shape[0] == 1:
    #            x = np.linspace(0, len(data), len(data[0]))
    #            y = [data[0]]
    #        if shape[0] >= 2:
    #            x = data[0]
    #            y = data[1:]
    #    self._set_real(np.isreal(y).all())
    #    for i, values in enumerate(y):
    #        self._display_curve_index(x, values, i)
    #    while (i + 1 < len(self.curves)):  # delete remaining curves
    #        i += 1
    #        self.curves[i].hide()

    #def _display_curve_index(self, x, values, i):
    #    y_mag = values if self._is_real else self._magnitude(values)
    #    y_phase = np.zeros(len(values)) if self._is_real else \
    #        self._phase(values)
    #    if len(self.curves)<=i:
    #        color = self._defaultcolors[i%len(self._defaultcolors)]
    #        self.curves.append(self.plot_item.plot(pen=color))
    #        self.curves_phase.append(self.plot_item_phase.plot(pen=color))
    #    self.curves[i].setData(x, y_mag)
    #    self.curves_phase[i].setData(x, y_phase)

    def _set_real(self, bool):
        self._is_real = bool
        if bool:
            self.plot_item_phase.hide()
            self.plot_item.setTitle("")
        else:
            self.plot_item_phase.show()
            self.plot_item.setTitle("Magnitude (dB)")


class CurveAttributeWidget(DataAttributeWidget):
    """
    Plots a curve (complex or real), with an id number as input.
    """
    def get_xy_data(self, new_value):
        """ helper function to extract xy data from a curve object"""
        if new_value is None:
            return None, None, None
        try:
            data = getattr(self.module, '_' + self.attribute_name + '_object').data
            name = getattr(
                self.module, '_' + self.attribute_name + '_object').params['name']
        except:
            return None, None, None
        else:
            x, y = data
            return x, y, name

    def _set_widget_value(self, new_value):
        x, y, name = self.get_xy_data(new_value)
        if x is not None:
            if not np.isreal(y).all():
                self.curve.setData(x, self._magnitude(y))
                self.curve_phase.setData(x, self._phase(y))
                self.plot_item_phase.show()
                self.plot_item.setTitle(name + " - Magnitude (dB)")
            else:
                self.curve.setData(x, np.real(y))
                self.plot_item_phase.hide()
                self.plot_item.setTitle(name)


class CurveSelectAttributeWidget(SelectAttributeWidget):
    """
    Select one or many curves.
    """
    def _make_widget(self):
        """
        Sets up the widget (here a QComboBox)

        :return:
        """
        self.widget = QtWidgets.QListWidget()
        self.widget.addItems(self.options)
        self.widget.currentItemChanged.connect(self.write_widget_value_to_attribute)

    def _get_widget_value(self):
        return int(self.widget.currentItem().text())

    def _set_widget_value(self, new_value):
        """ should be much simpler here. all this logic should be in the
        attribute """
        if new_value is None:
            new_value = -1
        if hasattr(new_value, 'pk'):
            new_value = new_value.pk
        try:
            index = self.options.index(str(new_value))
        except IndexError:
            self.module._logger.warning("SelectWidget %s could not find "
                                        "current value %s in the options %s",
                                        self.attribute_name, self.new_value, self.options)
            index = 0
        self.widget.setCurrentRow(index)
