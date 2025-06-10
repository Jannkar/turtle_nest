/* ------------------------------------------------------------------
 * Copyright 2025 Janne Karttunen
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * ------------------------------------------------------------------
*/

#include "test_utils.h"

#include <QFile>
#include <QDebug>
#include <QTemporaryDir>
#include <QXmlStreamReader>
#include <QRegularExpression>
#include <QProcess>
#include <QTimer>
#include <QDateTime>
#include <QStringList>
#include <QElapsedTimer>
#include <QThread>
#include <csignal>

bool file_exists(QString path) {
    QFile file(path);
    return file.exists();
}


bool string_exists_in_file(const QString& filePath, const QString& searchString) {
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qDebug() << "Error opening file:" << file.errorString();
        return false;
    }

    QTextStream in(&file);

    while (!in.atEnd()) {
        QString line = in.readLine();
        if (line.contains(searchString, Qt::CaseSensitive)) {
            file.close();
            return true;
        }
    }

    file.close();
    return false;
}


QString get_tmp_workspace_path(){
    QTemporaryDir temp_dir;
    return temp_dir.path() + "/ros2_ws/src";
}

bool dir_is_empty(const QString& dirPath) {
    QDir dir(dirPath);
    if (!dir.exists()) {
        qDebug() << "The directory does not exist.";
        throw std::runtime_error("Directory does not exist");
    }
    // Filter out "." and ".." which are always present in directories
    QStringList files = dir.entryList(QDir::NoDotAndDotDot | QDir::AllEntries);
    qInfo() << files;
    return files.isEmpty();
}

QString run_command(QString command, QStringList outputs_to_wait, QString workspace_path){

    QDateTime current_date_time = QDateTime::currentDateTime();
    qint64 timestamp_seconds = current_date_time.toSecsSinceEpoch();
    qint64 timestamp_ms = current_date_time.toMSecsSinceEpoch();
    QString formatted_time = QString("[%1.%2]")
                                 .arg(timestamp_seconds)
                                 .arg(timestamp_ms % 1000, 3, 10, QChar('0'));

    QProcess *process = new QProcess();
    process->setProcessChannelMode(QProcess::MergedChannels);

    // Source the workspace if given
    if (!workspace_path.isEmpty()) {
        QDir dir(workspace_path);
        dir.cdUp(); // Get the path without /src extension

        // We need to use setsid to set a process group for the command.
        // Otherwise SIGINT won't kill all the subprocesses that were started.
        // Without setting a process group, ros2 launch works when killing just
        // the main process, but ros2 run will leave a Node running on background.
        command = QString("setsid bash -c 'source %1/install/setup.bash && %2'")
                      .arg(dir.path(), command);
    }

    qDebug().noquote() << formatted_time << "Running:" << "bash -c" << command;
    process->start("bash", QStringList() << "-c" << command);

    // Wait for the command to execute until all "outputs_to_wait" strings are found from the output
    QElapsedTimer timer;
    timer.start();

    QString output;
    while (!outputs_to_wait.isEmpty() && timer.elapsed() < 10000) {
        if (!process->waitForReadyRead(100)) {
            continue;
        }
        QString new_output = process->readAllStandardOutput();
        output += new_output;
        qDebug().noquote() << new_output;

        // Check if any expected output is found in the output
        for (int i = 0; i < outputs_to_wait.size(); ) {
            if (output.contains(outputs_to_wait[i])) {
                outputs_to_wait.removeAt(i);
            } else {
                ++i;
            }
        }
    }

    if (outputs_to_wait.isEmpty()){
        qDebug() << "Found all the expected lines";
    } else {
        qDebug() << "Timed out while waiting for the expected lines";
    }
    // Sending SIGINT very soon after the launch was started will make the process hang. Wait for a second before doing that.
    QThread::sleep(1);
    qDebug() << "Sending SIGINT";
    kill(-process->processId(), SIGINT);  // '-' before the process kills the whole process group

    if (!process->waitForFinished(10000)) {
        // If it still didn't finish, terminate
        qCritical() << "Process didn't finish with SIGINT. Terminating. This might leave active background processes";
        process->terminate();
    }

    QString new_output = process->readAllStandardOutput();
    output += new_output;
    qDebug().noquote() << new_output;

    return output;
}
