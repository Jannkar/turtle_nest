#include "turtle_nest/addnodedialog.h"
#include "turtle_nest/node_type_enum.h"
#include "turtle_nest/string_tools.h"
#include "ui_addnodedialog.h"
#include <QPushButton>
#include <QDebug>
#include <QDir>
#include <turtle_nest/packageinfo.h>


AddNodeDialog::AddNodeDialog(QWidget *parent, const PackageInfo &package_info)
    : QDialog(parent)
    , ui(new Ui::AddNodeDialog)
    , pkg_info(package_info)
{
    ui->setupUi(this);
    setWindowTitle("Add New Node");
    ui->buttonBox->button(QDialogButtonBox::Ok)->setEnabled(false);

    if (pkg_info.package_type != CPP_AND_PYTHON){
        ui->nodeTypeBox->hide();
    }

    // Make the dialog window to have a smallest size possible, after hiding elements.
    this->layout()->activate();
    this->resize(this->minimumSizeHint());
    this->setFixedSize(this->size());
}

AddNodeDialog::~AddNodeDialog()
{
    delete ui;
}

void AddNodeDialog::on_nodeNameEdit_textEdited(const QString &arg1)
{
    autocorrect_line_edit(arg1, ui->nodeNameEdit);
    if (ui->nodeNameEdit->text().length() <= 1){
        ui->buttonBox->button(QDialogButtonBox::Ok)->setEnabled(false);
    }
    else {
        ui->buttonBox->button(QDialogButtonBox::Ok)->setEnabled(true);
    }
}

QString AddNodeDialog::get_node_name(){
    return ui->nodeNameEdit->text();
}

NodeType AddNodeDialog::get_node_type(){
    switch (pkg_info.package_type){
    case BuildType::CPP:
        return CPP_NODE;
    case BuildType::PYTHON:
        return PYTHON_NODE;
    case BuildType::CPP_AND_PYTHON:
        if (ui->cppButton->isChecked()){
            return CPP_NODE;
        }
        else if (ui->pythonButton->isChecked()){
            return PYTHON_NODE;
        }
        throw std::logic_error("Node was not selected. This shouldn't be possible.");
    default:
        throw std::logic_error("Node adding not implemented for this package type.");
    }
}
