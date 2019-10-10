#include "CoordinateFrameSetView.h"
#include "CoordinateFrameSetItem.h"
#include "PositionEditManager.h"
#include <cnoid/CoordinateFrameSet>
#include <cnoid/CoordinateFrameContainer>
#include <cnoid/ViewManager>
#include <cnoid/MenuManager>
#include <cnoid/TargetItemPicker>
#include <cnoid/Buttons>
#include <cnoid/EigenUtil>
#include <QBoxLayout>
#include <QLabel>
#include <QTableView>
#include <QHeaderView>
#include <QAbstractTableModel>
#include <QStyledItemDelegate>
#include <QKeyEvent>
#include <QMouseEvent>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

constexpr int NumColumns = 3;
constexpr int IdColumn = 0;
constexpr int NoteColumn = 1;
constexpr int PositionColumn = 2;

class FrameContainerModel : public QAbstractTableModel
{
public:
    CoordinateFrameContainerPtr frames;
    QFont monoFont;
    
    FrameContainerModel(QObject* parent);
    void setFrameContainer(CoordinateFrameContainer* frames);
    bool isValid() const { return frames != nullptr; }
    int numFrames() const { return frames ? frames->numFrames() : 0; }
    virtual int rowCount(const QModelIndex& parent) const override;
    virtual int columnCount(const QModelIndex& parent) const override;
    virtual QVariant headerData(int section, Qt::Orientation orientation, int role) const override;
    virtual QModelIndex index(int row, int column, const QModelIndex& parent = QModelIndex()) const override;
    virtual Qt::ItemFlags flags(const QModelIndex& index) const override;
    virtual QVariant data(const QModelIndex& index, int role) const override;
    virtual bool setData(const QModelIndex& index, const QVariant& value, int role) override;
    void addFrame(int row, CoordinateFrame* frame, bool doInsert);
    void removeFrames(QModelIndexList selected);
    void notifyFramePositionUpdate(CoordinateFrame* frame);
};

class CustomizedItemDelegate : public QStyledItemDelegate
{
public:
    CoordinateFrameSetView::Impl* view;
    
    CustomizedItemDelegate(CoordinateFrameSetView::Impl* view);
    virtual QWidget* createEditor(
        QWidget* parent, const QStyleOptionViewItem& option, const QModelIndex& index) const override;
    virtual void updateEditorGeometry(
        QWidget* editor, const QStyleOptionViewItem& option, const QModelIndex& index) const override;
    virtual void setEditorData(QWidget* editor, const QModelIndex& index) const override;
    virtual void setModelData(QWidget* editor, QAbstractItemModel* model, const QModelIndex& index) const override;
};

}

namespace cnoid {

class CoordinateFrameSetView::Impl : public QTableView, public AbstractPositionEditTarget
{
public:
    CoordinateFrameSetView* self;
    TargetItemPicker<CoordinateFrameSetItem> targetItemPicker;
    CoordinateFrameSetItemPtr targetItem;
    CoordinateFrameContainerPtr frameContainer;
    FrameContainerModel* frameContainerModel;
    QLabel targetLabel;
    PushButton addButton;
    MenuManager contextMenuManager;

    // For the position editing by external editors
    PositionEditManager* positionEditManager;
    CoordinateFramePtr frameBeingEditedOutside;
    Signal<void(const Position& T)> sigPositionChanged_;
    Signal<void()> sigPositionEditTargetExpired_;

    Impl(CoordinateFrameSetView* self);
    void setCoordinateFrameSetItem(CoordinateFrameSetItem* item);
    void addFrameIntoCurrentIndex(bool doInsert);
    void addFrame(int index, bool doInsert);
    void removeSelectedFrames();
    virtual void keyPressEvent(QKeyEvent* event) override;
    virtual void mousePressEvent(QMouseEvent* event) override;
    void showContextMenu(int row, QPoint globalPos);
    virtual void currentChanged(const QModelIndex& current, const QModelIndex& previous) override;

    void startExternalPositionEditing(CoordinateFrame* frame);
    virtual Referenced* getPositionObject() override;
    virtual std::string getPositionName() const override;
    virtual CoordinateFrameId getPreferredBaseFrameId() const override;
    virtual CoordinateFrameId getPreferredLocalFrameId() const override;
    virtual Position getGlobalPosition() const override;
    virtual bool setPosition(
        const Position& T_global,
        const Position& T_frame, CoordinateFrame* baseFrame, CoordinateFrame* localFrame) override;
    virtual SignalProxy<void(const Position& T)> sigGlobalPositionChanged() override;
    virtual SignalProxy<void()> sigPositionEditTargetExpired() override;
};

}


FrameContainerModel::FrameContainerModel(QObject* parent)
    : QAbstractTableModel(parent),
      monoFont("Monospace")
{
    monoFont.setStyleHint(QFont::TypeWriter);
}


void FrameContainerModel::setFrameContainer(CoordinateFrameContainer* frames)
{
    beginResetModel();
    this->frames = frames;
    endResetModel();
}


int FrameContainerModel::rowCount(const QModelIndex& parent) const
{
    int n = 0;
    if(!parent.isValid() && frames){
        n = frames->numFrames();
    }
    if(n == 0){ // to show an empty row
        n = 1;
    }
    return n;
}


int FrameContainerModel::columnCount(const QModelIndex& parent) const
{
    return NumColumns;
}
        

QVariant FrameContainerModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if(role == Qt::DisplayRole){
        if(orientation == Qt::Horizontal){
            switch(section){
            case IdColumn:
                return "ID";
            case NoteColumn:
                return _("Note");
            case PositionColumn:
                return _("Position");
            default:
                return QVariant();
            }
        } else {
            return QString::number(section);
        }
    } else if(role == Qt::TextAlignmentRole){
        if(orientation == Qt::Horizontal){
            return Qt::AlignCenter;
        }
    }
    return QVariant();
}


QModelIndex FrameContainerModel::index(int row, int column, const QModelIndex& parent) const
{
    if(!frames || parent.isValid()){
        return QModelIndex();
    }
    if(row < frames->numFrames()){
        auto frame = frames->frame(row);
        return createIndex(row, column, frame);
    }
    return QModelIndex();
}
    

Qt::ItemFlags FrameContainerModel::flags(const QModelIndex& index) const
{
    auto flags = QAbstractTableModel::flags(index);
    if(index.isValid() && index.column() != PositionColumn){
        flags = flags | Qt::ItemIsEditable;
    }
    return flags;
}


QVariant FrameContainerModel::data(const QModelIndex& index, int role) const
{
    auto frame = static_cast<CoordinateFrame*>(index.internalPointer());

    if(!frame || !index.isValid()){
        return QVariant();
    }

    int column = index.column();
    if(role == Qt::DisplayRole || role == Qt::EditRole){
        if(column == IdColumn){
            return frame->id().label().c_str();

        } else if(column == NoteColumn){
            return frame->note().c_str();

        } else if(column == PositionColumn){
            auto p = frame->T().translation();
            auto rpy = degree(rpyFromRot(frame->T().linear()));
            return format("( {0: 1.3f}, {1: 1.3f}, {2: 1.3f}, {3: 6.1f}, {4: 6.1f}, {5: 6.1f} )",
                          p.x(), p.y(), p.z(), rpy[0], rpy[1], rpy[2]).c_str();
        }
    } else if(role == Qt::TextAlignmentRole){
        if(column == NoteColumn){
            return (Qt::AlignLeft + Qt::AlignVCenter);
        } else {
            return Qt::AlignCenter;
        }
    } else if(role == Qt::FontRole){
        if(column == PositionColumn){
            return monoFont;
        }
    }
            
    return QVariant();
}


bool FrameContainerModel::setData(const QModelIndex& index, const QVariant& value, int role)
{
    if(index.isValid() && role == Qt::EditRole){
        auto frame = static_cast<CoordinateFrame*>(index.internalPointer());
        int column = index.column();
        if(column == IdColumn){
            bool isInt;
            auto stringId = value.toString();
            int intId = stringId.toInt(&isInt);
            if(isInt){
                frames->resetId(frame, intId);
            } else {
                frames->resetId(frame, stringId.toStdString());
            }
            Q_EMIT dataChanged(index, index, {role});
        } else if(column == NoteColumn){
            frame->setNote(value.toString().toStdString());
            Q_EMIT dataChanged(index, index, {role});
        }
    }
    return false;
}


void FrameContainerModel::addFrame(int row, CoordinateFrame* frame, bool doInsert)
{
    if(frames){
        if(frames->numFrames() == 0){
            // Remove the empty row first
            beginRemoveRows(QModelIndex(), 0, 0);
            endRemoveRows();
        }
        int newFrameRow = doInsert ? row : row + 1;
        beginInsertRows(QModelIndex(), newFrameRow, newFrameRow);
        frames->insert(newFrameRow, frame);
        endInsertRows();
    }
}


void FrameContainerModel::removeFrames(QModelIndexList selected)
{
    if(frames){
        std::sort(selected.begin(), selected.end());
        int numRemoved = 0;
        for(auto& index : selected){
            int row = index.row() - numRemoved;
            beginRemoveRows(QModelIndex(), row, row);
            frames->removeAt(row);
            ++numRemoved;
            endRemoveRows();
        }
        if(frames->numFrames() == 0){
            // This is necessary to show the empty row
            beginResetModel();
            endResetModel();
        }
    }
}


void FrameContainerModel::notifyFramePositionUpdate(CoordinateFrame* frame)
{
    if(frames){
        int row = frames->indexOf(frame);
        if(row >= 0){
            auto modelIndex = index(row, PositionColumn, QModelIndex());
            Q_EMIT dataChanged(modelIndex, modelIndex, { Qt::EditRole });
        }
    }
}
        

CustomizedItemDelegate::CustomizedItemDelegate(CoordinateFrameSetView::Impl* view)
    : QStyledItemDelegate(view),
      view(view)
{

}


QWidget* CustomizedItemDelegate::createEditor
(QWidget* parent, const QStyleOptionViewItem& option, const QModelIndex& index) const
{
    QWidget* editor = nullptr;
    
    auto frame = static_cast<CoordinateFrame*>(index.internalPointer());
    
    switch(index.column()){
    case IdColumn:
    default:
        editor = QStyledItemDelegate::createEditor(parent, option, index);
        break;
    }

    return editor;
}


void CustomizedItemDelegate::updateEditorGeometry
(QWidget* editor, const QStyleOptionViewItem& option, const QModelIndex& index) const
{
    editor->setGeometry(option.rect);
}


void CustomizedItemDelegate::setEditorData(QWidget* editor, const QModelIndex& index) const
{
    QStyledItemDelegate::setEditorData(editor, index);
}


void CustomizedItemDelegate::setModelData(QWidget* editor, QAbstractItemModel* model, const QModelIndex& index) const
{
    switch(index.column()){
    default:
        QStyledItemDelegate::setModelData(editor, model, index);
    }
}


void CoordinateFrameSetView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<CoordinateFrameSetView>(
        "CoordinateFrameSetView", N_("Coordinate Systems"), ViewManager::SINGLE_OPTIONAL);
}


CoordinateFrameSetView::CoordinateFrameSetView()
{
    impl = new Impl(this);
}


CoordinateFrameSetView::Impl::Impl(CoordinateFrameSetView* self)
    : self(self),
      targetItemPicker(self)
{
    self->setDefaultLayoutArea(View::RIGHT);

    auto vbox = new QVBoxLayout;
    vbox->setSpacing(0);

    int hs = self->style()->pixelMetric(QStyle::PM_LayoutHorizontalSpacing);
    
    auto hbox = new QHBoxLayout;
    hbox->setSpacing(0);
    hbox->addSpacing(hs);
    targetLabel.setStyleSheet("font-weight: bold");
    hbox->addWidget(&targetLabel, 0, Qt::AlignVCenter);
    hbox->addSpacing(hs);
    addButton.setText(_("Add"));
    addButton.sigClicked().connect([&](){ addFrameIntoCurrentIndex(false); });
    hbox->addWidget(&addButton);
    hbox->addStretch();
    vbox->addLayout(hbox);

    // Setup the table
    auto hframe = new QFrame;
    hframe->setFrameStyle(QFrame::HLine | QFrame::Sunken);
    vbox->addWidget(hframe);
    setFrameShape(QFrame::NoFrame);
    setSelectionBehavior(QAbstractItemView::SelectRows);
    setSelectionMode(QAbstractItemView::ExtendedSelection);
    setTabKeyNavigation(true);
    setCornerButtonEnabled(true);
    setHorizontalScrollMode(QAbstractItemView::ScrollPerPixel);
    setItemDelegate(new CustomizedItemDelegate(this));
    setEditTriggers(
        /* QAbstractItemView::CurrentChanged | */
        QAbstractItemView::DoubleClicked |
        /* QAbstractItemView::SelectedClicked | */
        QAbstractItemView::EditKeyPressed |
        QAbstractItemView::AnyKeyPressed);

    frameContainerModel = new FrameContainerModel(this);
    setModel(frameContainerModel);

    auto hheader = horizontalHeader();
    hheader->setSectionResizeMode(IdColumn, QHeaderView::ResizeToContents);
    hheader->setSectionResizeMode(NoteColumn, QHeaderView::Stretch);
    hheader->setSectionResizeMode(PositionColumn, QHeaderView::ResizeToContents);
    auto vheader = verticalHeader();
    vheader->setSectionResizeMode(QHeaderView::ResizeToContents);
    vheader->hide();

    connect(this, SIGNAL(clicked(const QModelIndex&)),
            self, SLOT(onTableItemClicked(const QModelIndex&)));
    
    vbox->addWidget(this);
    self->setLayout(vbox);

    targetItemPicker.sigTargetItemChanged().connect(
        [&](CoordinateFrameSetItem* item){
            setCoordinateFrameSetItem(item); });

    positionEditManager = PositionEditManager::instance();
}


CoordinateFrameSetView::~CoordinateFrameSetView()
{
    delete impl;
}


void CoordinateFrameSetView::Impl::setCoordinateFrameSetItem(CoordinateFrameSetItem* item)
{
    targetItem = item;

    if(item){
        targetLabel.setText(item->name().c_str());
        frameContainer = item->frames();
        frameContainerModel->setFrameContainer(frameContainer);
    } else {
        targetLabel.setText("---");
        frameContainer = nullptr;
        frameContainerModel->setFrameContainer(nullptr);
    }
    addButton.setEnabled(targetItem != nullptr);
}


void CoordinateFrameSetView::Impl::addFrameIntoCurrentIndex(bool doInsert)
{
    auto current = selectionModel()->currentIndex();
    int index = current.isValid() ? current.row() : frameContainerModel->numFrames();
    addFrame(index, doInsert);
}


void CoordinateFrameSetView::Impl::addFrame(int index, bool doInsert)
{
    if(frameContainer){
        auto id = frameContainer->createNextId();
        CoordinateFramePtr frame = new CoordinateFrame(id);
        frameContainerModel->addFrame(index, frame, doInsert);
        resizeColumnToContents(IdColumn);
        resizeColumnToContents(PositionColumn);
    }
}


void CoordinateFrameSetView::Impl::removeSelectedFrames()
{
    frameContainerModel->removeFrames(selectionModel()->selectedRows());
}


void CoordinateFrameSetView::Impl::keyPressEvent(QKeyEvent* event)
{
    bool processed = true;

    switch(event->key()){
    case Qt::Key_Escape:
        clearSelection();
        break;
    case Qt::Key_Insert:
        addFrameIntoCurrentIndex(true);
        break;
    case Qt::Key_Delete:
        removeSelectedFrames();
        break;
    default:
        processed = false;
        break;
    }
        
    if(!processed && (event->modifiers() & Qt::ControlModifier)){
        processed = true;
        switch(event->key()){
        case Qt::Key_A:
            selectAll();
            break;
        default:
            processed = false;
            break;
        }
    }

    if(!processed){
        QTableView::keyPressEvent(event);
    }
}

       
void CoordinateFrameSetView::Impl::mousePressEvent(QMouseEvent* event)
{
    QTableView::mousePressEvent(event);

    if(event->button() == Qt::RightButton){
        int row = rowAt(event->pos().y());
        if(row >= 0){
            showContextMenu(row, event->globalPos());
        }
    }
}


void CoordinateFrameSetView::Impl::showContextMenu(int row, QPoint globalPos)
{
    contextMenuManager.setNewPopupMenu(this);

    contextMenuManager.addItem(_("Add"))
        ->sigTriggered().connect([=](){ addFrame(row, false); });

    contextMenuManager.addItem(_("Remove"))
        ->sigTriggered().connect([=](){ removeSelectedFrames(); });
    
    contextMenuManager.popupMenu()->popup(globalPos);
}


void CoordinateFrameSetView::Impl::currentChanged(const QModelIndex& current, const QModelIndex& previous)
{
    QTableView::currentChanged(current, previous);
    
    auto frame = static_cast<CoordinateFrame*>(current.internalPointer());
    startExternalPositionEditing(frame);
}


void CoordinateFrameSetView::onTableItemClicked(const QModelIndex& index)
{
    auto frame = static_cast<CoordinateFrame*>(index.internalPointer());
    impl->startExternalPositionEditing(frame);
}


void CoordinateFrameSetView::Impl::startExternalPositionEditing(CoordinateFrame* frame)
{
    if(frame != frameBeingEditedOutside && frameBeingEditedOutside){
        sigPositionEditTargetExpired_();
    }

    frameBeingEditedOutside = frame;

    if(frame){
        positionEditManager->requestPositionEdit(this);
    }
}


Referenced* CoordinateFrameSetView::Impl::getPositionObject()
{
    return frameBeingEditedOutside;
}


std::string CoordinateFrameSetView::Impl::getPositionName() const
{
    if(frameBeingEditedOutside){
        return format("{0}: {1}", targetItem->name(), frameBeingEditedOutside->id().label());
    }
    return string();
}


CoordinateFrameId CoordinateFrameSetView::Impl::getPreferredBaseFrameId() const
{
    return CoordinateFrameId::defaultId();
}


CoordinateFrameId CoordinateFrameSetView::Impl::getPreferredLocalFrameId() const
{
    return CoordinateFrameId::defaultId();
}


Position CoordinateFrameSetView::Impl::getGlobalPosition() const
{
    if(frameBeingEditedOutside){
        return frameBeingEditedOutside->T();
    }
    return Position::Identity();
}


bool CoordinateFrameSetView::Impl::setPosition
(const Position& T_global,
 const Position& /* T_frame */, CoordinateFrame* /* baseFrame */, CoordinateFrame* /* localFrame */)
{
    if(frameBeingEditedOutside){
        frameBeingEditedOutside->T() = T_global;
        frameContainerModel->notifyFramePositionUpdate(frameBeingEditedOutside);
        sigPositionChanged_(T_global);
        return true;
    }
    return false;
}


SignalProxy<void(const Position& T)> CoordinateFrameSetView::Impl::sigGlobalPositionChanged()
{
    return sigPositionChanged_;
}


SignalProxy<void()> CoordinateFrameSetView::Impl::sigPositionEditTargetExpired()
{
    return sigPositionEditTargetExpired_;
}


bool CoordinateFrameSetView::storeState(Archive& archive)
{
    impl->targetItemPicker.storeTargetItem(archive, "currentItem");
    return true;
}


bool CoordinateFrameSetView::restoreState(const Archive& archive)
{
    impl->targetItemPicker.restoreTargetItemLater(archive, "currentItem");
    return true;
}
