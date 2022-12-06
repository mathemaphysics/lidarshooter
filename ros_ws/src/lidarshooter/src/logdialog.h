#ifndef LOGDIALOG_H
#define LOGDIALOG_H

#include <QDialog>
#include <QTextEdit>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/qt_sinks.h>

namespace Ui {
class LogDialog;
}

class LogDialog : public QDialog
{
    Q_OBJECT

public:
    explicit LogDialog(QWidget *parent = nullptr);
    ~LogDialog();

    /**
     * @brief Get the log output top window object
     * 
     * @return QTextEdit* Return pointer to the top
     */
    QTextEdit* getTextEditTop();

    /**
     * @brief Get the log output bottom window object
     * 
     * @return QTextEdit* Return pointer to the bottom
     */
    QTextEdit* getTextEditBottom();

private:
    Ui::LogDialog *ui;
};

#endif // LOGDIALOG_H
