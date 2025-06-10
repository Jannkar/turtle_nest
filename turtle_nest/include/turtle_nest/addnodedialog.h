#ifndef ADDNODEDIALOG_H
#define ADDNODEDIALOG_H

#include "turtle_nest/node_type_enum.h"
#include "turtle_nest/packageinfo.h"
#include <QDialog>

namespace Ui {
class AddNodeDialog;
}

class AddNodeDialog : public QDialog
{
    Q_OBJECT

public:
    explicit AddNodeDialog(QWidget *parent, const PackageInfo &package_info);
    ~AddNodeDialog();

    QString get_node_name();
    NodeType get_node_type();

private slots:
    void on_nodeNameEdit_textEdited(const QString &arg1);

private:
    Ui::AddNodeDialog *ui;
    PackageInfo pkg_info;
};

#endif // ADDNODEDIALOG_H
