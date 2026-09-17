"""Collapsible group boxes for attribute widgets.

Any attribute (property or register) declared with a non-empty ``group``
argument, e.g.::

    swing_rate_hz = FloatProperty(min=0.1, max=50.0, default=5.0,
                                  group='Scanner360', doc='...')

is not added to its module widget's attribute layout directly: it goes
inside a :class:`CollapsibleGroupBox` titled with the group name, created on
first use at the place where the first attribute of that group would have
gone. Clicking the box header folds the whole set of knobs away; the
collapsed/expanded state of every box is saved in the module's config branch
and restored at the next start.

The boxes are placed after the ungrouped widgets, in the order given by the
module's optional ``_gui_groups`` list (names it does not mention keep
first-use order). Groups start COLLAPSED on a config that has never seen
them, so a fresh pane opens showing its ungrouped knobs only; list a group in
the widget's ``_default_expanded_groups`` (or clear
``_groups_collapsed_by_default``) to open it instead.

Module widgets get this for free through :class:`AttributeGroupMixin`, which
:class:`~pyrpl.widgets.module_widgets.base_module_widget.ReducedModuleWidget`
already mixes in. A widget that builds its own attribute panel (rather than
using ``init_attribute_layout``) has to call ``_add_attribute_widget``
instead of ``self.attribute_layout.addWidget`` and ``_place_attribute_groups``
once the loop is done, and may override ``_group_content_layout`` /
``_add_group_box`` to control what a box contains and where it is placed.
"""
import logging
from collections import OrderedDict

from qtpy import QtCore, QtWidgets

from . import escape_mnemonics

logger = logging.getLogger(name=__name__)


class CollapsibleGroupBox(QtWidgets.QFrame):
    """A titled frame whose header arrow folds its content away.

    Not a checkable QGroupBox on purpose: Qt enables/disables all children of
    a checkable group box along with the check mark, which would clobber any
    per-widget enabling the module widget does (greying out the knobs a mode
    does not use).

    `content_layout` is the layout the grouped attribute widgets are added to
    (a plain horizontal box by default); it lives in `self.content`, which is
    simply hidden while the box is collapsed.
    """
    collapse_changed = QtCore.Signal(bool)      # True = now collapsed
    _notify = True                              # muted while constructing

    def __init__(self, title, parent=None, content_layout=None,
                 collapsed=False):
        super(CollapsibleGroupBox, self).__init__(parent)
        self._title = str(title)
        self.setFrameShape(QtWidgets.QFrame.StyledPanel)

        self.header = QtWidgets.QToolButton(self)
        self.header.setCheckable(True)
        self.header.setChecked(True)            # checked = expanded
        self.header.setAutoRaise(True)
        self.header.setFocusPolicy(QtCore.Qt.NoFocus)
        self.header.setToolButtonStyle(QtCore.Qt.ToolButtonTextBesideIcon)
        self._settings = []                     # (name, one-line brief)
        self.header.setSizePolicy(QtWidgets.QSizePolicy.Maximum,
                                  QtWidgets.QSizePolicy.Fixed)
        font = self.header.font()
        font.setBold(True)
        self.header.setFont(font)

        self.content = QtWidgets.QWidget(self)
        if content_layout is None:
            content_layout = QtWidgets.QHBoxLayout()
            content_layout.setContentsMargins(0, 0, 0, 0)
        self.content_layout = content_layout
        self.content.setLayout(content_layout)

        box = QtWidgets.QVBoxLayout(self)
        box.setContentsMargins(4, 2, 4, 4)
        box.setSpacing(2)
        box.addWidget(self.header, 0, QtCore.Qt.AlignLeft)
        box.addWidget(self.content)

        # let a flow/height-for-width layout inside the box drive our height
        if content_layout.hasHeightForWidth():
            for w in (self.content, self):
                policy = w.sizePolicy()
                policy.setHeightForWidth(True)
                w.setSizePolicy(policy)

        self.header.toggled.connect(self._on_header_toggled)
        self.set_collapsed(collapsed, notify=False)

    # ---- contents ----------------------------------------------------------
    def add_setting(self, name, brief=''):
        """Record a setting that went into this box, so the header can name
        its contents (they are invisible while it is folded)."""
        self._settings.append((str(name), str(brief or '')))
        self._refresh_header()

    def _tooltip(self):
        n = len(self._settings) or self.content_layout.count()
        lines = ['%s - %d setting%s, click to %s:'
                 % (self._title, n, '' if n == 1 else 's',
                    'unfold' if self.collapsed else 'fold')]
        for name, brief in self._settings:
            if brief:
                room = max(40, self._TIP_WIDTH - len(name) - 3)
                if len(brief) > room:
                    cut = brief[:room].rsplit(' ', 1)[0]
                    brief = (cut if len(cut) > room // 2 else brief[:room]) + '...'
                lines.append('    %s - %s' % (name, brief))
            else:
                lines.append('    %s' % name)
        return self._rich_text('\n'.join(lines))

    @staticmethod
    def _rich_text(text):
        """A tooltip is auto-detected as HTML when it happens to contain
        something tag-shaped - and some of these docs do ('<laser_sel>', a
        '# range +-<x>V' header). Escape it and mark it up ourselves, so
        every doc renders literally, line breaks and indent included, no
        matter what it holds."""
        esc = (text.replace('&', '&amp;').replace('<', '&lt;')
                   .replace('>', '&gt;'))
        rows = []
        for line in esc.split('\n'):
            body = line.lstrip(' ')
            rows.append('&nbsp;' * (len(line) - len(body)) + body)
        return '<div style="white-space:pre-wrap">%s</div>' % '<br>'.join(rows)

    _TIP_WIDTH = 96          # characters, before a brief is cut short

    # ---- collapsed state ---------------------------------------------------
    @property
    def collapsed(self):
        return not self.header.isChecked()

    def set_collapsed(self, collapsed, notify=True):
        collapsed = bool(collapsed)
        if collapsed == self.collapsed:
            self._refresh_header()
            return
        # _on_header_toggled does the work (and emits unless muted)
        self._notify = notify
        try:
            self.header.setChecked(not collapsed)
        finally:
            self._notify = True

    def _on_header_toggled(self, checked):
        self.content.setVisible(checked)
        self._refresh_header()
        self.updateGeometry()
        if self._notify:
            self.collapse_changed.emit(not checked)

    def flow_full_row(self):
        """Hook for a flow layout that gives grouped boxes a row of their
        own: only an EXPANDED box needs the full width. Collapsed we are just
        a header, and pack alongside the other items like any chip."""
        return not self.collapsed

    def _refresh_header(self):
        """Arrow + title; a collapsed box also shows how many knobs it hides,
        so a folded group does not look like an empty label. The tooltip
        lists them by name whichever way the box stands."""
        if self.collapsed:
            self.header.setArrowType(QtCore.Qt.RightArrow)
            n = len(self._settings) or self.content_layout.count()
            text = '%s  (%d)' % (self._title, n) if n else self._title
        else:
            self.header.setArrowType(QtCore.Qt.DownArrow)
            text = self._title
        # a button's text is mnemonic markup ('& band' would underline the
        # b and eat the ampersand)
        self.header.setText(escape_mnemonics(text))
        self.header.setToolTip(self._tooltip())


class AttributeGroupMixin(object):
    """Adds collapsible attribute groups to a module widget.

    Mixed into ReducedModuleWidget, so every module widget supports it. The
    hooks a subclass may override are `_group_content_layout` (the layout
    built inside a new box) and `_add_group_box` (where the box is placed).
    """
    # config key holding {group name: collapsed} in the module's branch
    _COLLAPSED_GROUPS_KEY = 'gui_collapsed_groups'
    # a group the config has never seen starts folded, so a fresh pane opens
    # showing only its ungrouped knobs
    _groups_collapsed_by_default = True
    _default_expanded_groups = ()           # exceptions to that

    def _init_attribute_groups(self):
        self.attribute_groups = OrderedDict()   # group name -> group box
        self._placed_groups = set()             # boxes already in the layout

    def _attribute_group_of(self, attribute, name=None):
        """The group an attribute (descriptor or callable) belongs to, '' for
        the ungrouped ones.

        The module's optional `_gui_attribute_groups` {name: group} map wins
        over the attribute's own `group=`: it is the way to group a setting
        INHERITED from a base module, whose descriptor is shared with every
        other module of that base and must not be tagged in place."""
        override = getattr(self.module, '_gui_attribute_groups', None) or {}
        if name is not None and name in override:
            return str(override[name] or '').strip()
        return str(getattr(attribute, 'group', '') or '').strip()

    @staticmethod
    def _attribute_brief(attribute, name=None):
        """One line describing an attribute, for the group box tooltip: the
        first line of its doc, whitespace collapsed. `doc` first (a
        SelectProperty's __doc__ appends its option list), then __doc__ (which
        is where BaseProperty keeps it, and a callable its docstring)."""
        doc = getattr(attribute, 'doc', None)
        if not isinstance(doc, str) or not doc:
            doc = getattr(attribute, '__doc__', '') or ''
        if not isinstance(doc, str):
            return ''
        first = doc.replace('\r\n', '\n').replace('\r', '\n').split('\n')[0]
        return ' '.join(first.split())

    def _add_attribute_widget(self, widget, attribute=None, group=None,
                              name=None):
        """Add an attribute widget to the attribute layout, or to the
        collapsible box of its group when it declares one."""
        if group is None:
            group = self._attribute_group_of(attribute, name)
        if not group:
            self.attribute_layout.addWidget(widget)
            return
        box = self._attribute_group_box(group)
        box.content_layout.addWidget(widget)
        if name:
            # the header names what it hides (the knobs are invisible while
            # the box is folded)
            box.add_setting(name, self._attribute_brief(attribute, name))

    def _attribute_group_box(self, group):
        """The box holding `group`, created (and placed) on first use."""
        if not hasattr(self, 'attribute_groups'):
            self._init_attribute_groups()
        box = self.attribute_groups.get(group)
        if box is None:
            box = CollapsibleGroupBox(
                group, parent=None,
                content_layout=self._group_content_layout(),
                collapsed=self._group_starts_collapsed(group))
            box.collapse_changed.connect(self._attribute_group_toggled)
            self.attribute_groups[group] = box
        return box

    def _group_order(self):
        """The order the boxes are placed in: the module's optional
        `_gui_groups` list first, then any group it does not mention, in
        first-use order."""
        wanted = [g for g in (getattr(self.module, '_gui_groups', None) or ())
                  if g in self.attribute_groups]
        return wanted + [g for g in self.attribute_groups if g not in wanted]

    def _place_attribute_groups(self):
        """Put the boxes in the attribute layout, after the ungrouped
        widgets and in group order. Call this once the attribute widgets have
        been created - a box is built lazily but placed only from here, so
        the module's `_gui_groups` order wins over first use."""
        for name in self._group_order():
            if name not in self._placed_groups:
                self._placed_groups.add(name)
                self._add_group_box(self.attribute_groups[name])

    def showEvent(self, event):
        """Safety net: a custom attribute panel that fills in
        `_add_attribute_widget` but forgets `_place_attribute_groups` would
        show none of its boxes at all - the grouped knobs would simply
        vanish. Place whatever is still unplaced before the widget appears."""
        try:
            groups = getattr(self, 'attribute_groups', None)
            if groups and len(self._placed_groups) < len(groups):
                logger.debug('%s: placing %d attribute group box(es) from '
                             'showEvent - _place_attribute_groups was never '
                             'called', type(self).__name__,
                             len(groups) - len(self._placed_groups))
                self._place_attribute_groups()
        except Exception:
            logger.debug('%s: late group placement failed',
                         type(self).__name__, exc_info=True)
        super(AttributeGroupMixin, self).showEvent(event)

    # ---- overridable hooks --------------------------------------------------
    def _group_content_layout(self):
        """The layout built inside a new group box."""
        layout = QtWidgets.QHBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        return layout

    def _add_group_box(self, box):
        """Place a freshly created group box in the attribute layout."""
        self.attribute_layout.addWidget(box)

    def _attribute_group_toggled(self, collapsed):
        """A box was folded or unfolded: persist the new state."""
        self._save_collapsed_groups()

    # ---- persistence --------------------------------------------------------
    def _group_starts_collapsed(self, group):
        saved = self._saved_collapsed_groups()
        if group in saved:
            return bool(saved[group])          # what the user left it at
        if group in self._default_expanded_groups:
            return False
        return bool(self._groups_collapsed_by_default)

    def _saved_collapsed_groups(self):
        """{group: collapsed} as last saved in the module's config branch."""
        try:
            module = self.module
            # do not create an empty config section just to read the state
            if module.c is None or module.name not in module.parent.c:
                return {}
            saved = module.c._data.get(self._COLLAPSED_GROUPS_KEY, None)
            return dict(saved) if isinstance(saved, dict) else {}
        except Exception:
            logger.debug('%s: collapsed-group state not readable',
                         type(self).__name__, exc_info=True)
            return {}

    def _save_collapsed_groups(self):
        state = {name: bool(box.collapsed)
                 for name, box in getattr(self, 'attribute_groups',
                                          {}).items()}
        try:
            self.module.c[self._COLLAPSED_GROUPS_KEY] = state
        except Exception:
            logger.debug('%s: collapsed-group state not saved',
                         type(self).__name__, exc_info=True)
