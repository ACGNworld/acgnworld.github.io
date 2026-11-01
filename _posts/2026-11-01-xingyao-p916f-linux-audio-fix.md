---
layout: post
title: "机械革命星耀 14（P916F-STX）Linux 麦克风与扬声器修复及有线耳机切换"
date: 2026-11-01
tags: [Linux, 音频]
comments: true
author: 嘉余
---

> 本文记录一台机械革命星耀 14（XINGYAO Series-P916F-STX）在 Ubuntu Linux 下的音频修复过程，供同型号或硬件配置相近的设备参考。
>
> **重要：不要直接照抄本文中的 PCI 地址、声卡编号或 HDA 节点。** 需要先确认自己的机器使用相同的 Realtek ALC256 和 subsystem ID。`0x1B` 的修复命令只在本文硬件上验证过。

## 1. 硬件和软件环境

本文实测环境：

```text
机型：XINGYAO Series-P916F-STX
系统：Ubuntu 26.04.1 LTS
内核：Linux 7.0.0-42-generic
声卡：Realtek ALC256 + AMD ACP-PDM
PipeWire：1.6.2
WirePlumber：0.5.13
```

确认机型：

```bash
cat /sys/class/dmi/id/product_name
cat /sys/class/dmi/id/board_name
```

确认声卡：

```bash
cat /proc/asound/cards
aplay -l
arecord -l
```

本文机器的相关声卡如下：

```text
card 1: Generic_1 [HD-Audio Generic]
    device 0: ALC256 Analog
card 2: acppdmmach [acp-pdm-mach]
    device 0: DMIC capture dmic-hifi-0
```

确认 codec 和 subsystem ID：

```bash
grep -H -E '^(Codec:|Vendor Id:|Subsystem Id:)' \
  /proc/asound/card*/codec#0
```

本文目标硬件必须包含：

```text
Codec: Realtek ALC256
Vendor Id: 0x10ec0256
Subsystem Id: 0x1d05e004
```

如果 subsystem ID 不同，不要直接使用本文的 `0x1B` 修复命令。

## 2. 故障表现

本文机器原本有两个相互关联但独立的问题：

1. 默认数字麦克风可以显示输入电平，但实际录音没有有效人声，或者只有开头的“嗒”声。
2. 内置扬声器音量偏小、低频不足、声音偏薄。
3. 3.5mm 耳机不总能被正确识别。原先已经通过脚本切换 codec 输出开关来绕过这个问题。

默认 HiFi/UCM 配置同时暴露了两条录音路径：

```text
AMD ACP-PDM：acppdmmach
Realtek ALC256 HDA：Generic_1
```

本次排查中，默认 ACP-PDM 路径未获得可用录音；改为 ALC256 HDA 录音路径并调整增益后，使用者反馈麦克风改善。这里没有对 ACP-PDM 做原始样本级根因分析，不能据此断言所有同型号机器的数字麦克风都存在同一故障。

此外，Realtek codec 的 `0x1B` 节点默认 pin 配置标记为未连接，运行时处于输入、输出放大器静音状态。结合参考文章与本机启用后的实际试听，确认该节点用于缺失的第二组内置扬声器。只使用第一组 `0x14` 时，声音会比较小、低频不足。

### 开始前：工具、备份和硬件核对

Ubuntu 可安装以下排查工具：

```bash
sudo apt install alsa-utils alsa-tools pulseaudio-utils
command -v hda-verb amixer alsactl pactl wpctl
```

`pulseaudio-utils` 在这里仅用于提供 `pactl` 客户端，不需要把 PipeWire 换成 PulseAudio。`wpctl` 由 WirePlumber 提供；耳机脚本中的通知命令 `notify-send` 是可选功能，Ubuntu 可通过 `libnotify-bin` 安装。

改配置前先备份，尤其不要覆盖已有耳机脚本或其他人的音频规则：

```bash
backup="$HOME/audio-backup-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$backup"
cp -a ~/.config/wireplumber "$backup/user-wireplumber" 2>/dev/null || true
cp -a ~/.local/bin/toggle-audio.sh "$backup/" 2>/dev/null || true
sudo cp -a /etc/wireplumber "$backup/system-wireplumber" 2>/dev/null || true
alsactl -f "$backup/alsa.state" store Generic_1
pactl list cards > "$backup/cards.txt"
pactl list sinks > "$backup/sinks.txt"
pactl list sources > "$backup/sources.txt"
cat /sys/class/sound/hwC1D0/user_pin_configs > "$backup/user_pin_configs.txt"
```

这里的 `Generic_1`、`hwC1D0` 必须先核对再使用。如果 `user_pin_configs` 中已有其他设置，不要直接使用后文的安装示例：应先确认已有配置的用途，并合并、测试，而不是覆盖它们。后文 helper 面向本机原先没有自定义 pin 配置的情况。

## 3. 先确认当前 WirePlumber 版本

本文方案使用 WirePlumber 0.5 的配置格式：

```bash
wireplumber --version
```

如果是 WirePlumber 0.4，需要使用旧式 Lua 配置，不能混用本文的 `.conf` 配置。WirePlumber 0.5 的用户配置目录是：

```text
~/.config/wireplumber/wireplumber.conf.d/
```

系统级配置目录是：

```text
/etc/wireplumber/wireplumber.conf.d/
```

如果登录管理器或登录界面也会创建音频会话，建议使用系统级配置。

全文涉及的重启音频服务、重新解析 codec 都可能短暂中断声音，请先暂停会议、录音或重要播放。

## 4. 绕过错误的 ACP-PDM 路径

创建系统级 WirePlumber 配置：

```bash
sudo install -d -m 0755 /etc/wireplumber/wireplumber.conf.d
sudoedit /etc/wireplumber/wireplumber.conf.d/51-xingyao-audio.conf
```

填入以下内容。先用 `pactl list cards short` 确认设备名。本文机器是 `alsa_card.pci-0000_c2_00.6`，参考文章中的机器是不同 PCI 地址，`device.name` 和 `node.name` 均需要按本机输出替换。不要在用户级和系统级重复放相同规则；已有的同名文件应先备份、比较再修改。

```ini
# XINGYAO P916F-STX / ALC256 / subsystem 1d05:e004
monitor.alsa.rules = [
  {
    matches = [
      { device.name = "alsa_card.pci-0000_c2_00.6" }
    ]
    actions = {
      update-props = {
        # 绕过坏掉的 ACP-PDM/UCM 路径，直接枚举 HDA PCM。
        api.alsa.use-acp = false
        # 让 PipeWire 音量控制使用软件音量，硬件增益由 amixer 固定到 0 dB。
        api.alsa.soft-mixer = true
      }
    }
  }
  {
    matches = [
      { node.name = "alsa_output.pci-0000_c2_00.6.playback.0.0" }
    ]
    actions = {
      update-props = {
        # 原始 HDA 节点在本机内核上错误地报告了多声道能力。
        # 强制使用正常的前左/前右双声道。
        audio.channels = 2
        audio.position = [ FL FR ]
      }
    }
  }
]
```

重启 WirePlumber：

```bash
systemctl --user restart wireplumber.service
```

检查是否已经切换到原始 HDA 节点：

```bash
pactl list short sinks
pactl list short sources
pactl list cards
```

本文机器切换后的节点名为：

```text
alsa_output.pci-0000_c2_00.6.playback.0.0
alsa_input.pci-0000_c2_00.6.capture.0.0
```

节点编号可能不同，不能写死 `403`、`404` 等数字。

设置默认输入和输出：

```bash
pactl set-default-sink alsa_output.pci-0000_c2_00.6.playback.0.0
pactl set-default-source alsa_input.pci-0000_c2_00.6.capture.0.0
```

## 5. 临时恢复音量和麦克风增益

本文机器使用 ALSA 名称 `Generic_1`，也可以使用声卡数字编号，但数字编号可能因重启和插拔改变，因此更推荐使用稳定名称：

```bash
amixer -c Generic_1 sset Master 100% unmute
amixer -c Generic_1 sset Speaker 100% unmute
amixer -c Generic_1 sset Headphone 100% mute
amixer -c Generic_1 sset 'Auto-Mute Mode' Disabled
```

本文机器最终使用的麦克风增益是：

```bash
amixer -c Generic_1 sset Capture 15dB cap
amixer -c Generic_1 sset 'Internal Mic Boost' 20dB
```

如果录音出现削波、爆音或底噪过大，应逐步降低这两个增益。不要盲目使用最大值。

PipeWire 的桌面音量仍然可以单独调节，例如：

```bash
wpctl set-volume @DEFAULT_SINK@ 0.65
wpctl set-mute @DEFAULT_SINK@ 0
```

## 6. 验证 `0x1B` 第二组扬声器

先确认 HDA 设备路径。本文机器是 `/dev/snd/hwC1D0`，但 `C1D0` 不保证在其他机器上相同：

```bash
ls -l /dev/snd/hwC*D*
```

再次确认硬件身份：

```bash
grep -E '^(Vendor Id:|Subsystem Id:)' /proc/asound/card1/codec#0
```

仅当输出确实为本文目标硬件时，才执行：

```bash
sudo hda-verb /dev/snd/hwC1D0 0x1b SET_PIN_WIDGET_CONTROL 0x40
sudo hda-verb /dev/snd/hwC1D0 0x1b SET_AMP_GAIN_MUTE 0xb000
```

两个命令分别完成：

```text
0x1B -> 输出模式
0x1B -> 解除输出放大器静音
```

检查状态：

```bash
sed -n '/^Node 0x1b /,/^Node 0x1c /p' \
  /proc/asound/card1/codec#0
```

目标状态包括：

```text
Amp-Out vals:  [0x00 0x00]
Pin-ctls: 0x40: OUT
EAPD 0x2: EAPD
Connection:
    0x02* 0x03
```

### 为什么一次性 `hda-verb` 可能不够

在本文机器上，单独执行 `hda-verb` 后，再次播放时观察到 `0x1B` 回到了输入。推测与 codec/驱动重新初始化有关，但没有追踪到具体的内核写入调用，不能把复位时机或内核调用链视为已证实的结论。本机随后通过 `user_pin_configs` 让驱动将其作为扬声器管理：

```bash
sudo sh -c 'printf "0x1b 0x90170120\n" > /sys/class/sound/hwC1D0/user_pin_configs'
sudo sh -c 'echo 1 > /sys/class/sound/hwC1D0/reconfig'
```

`0x90170120` 把 `0x1B` 声明为内部模拟扬声器输出。命令是示意顺序，不建议在正在使用的桌面中直接照抄。执行 `reconfig` 前必须确保没有 PipeWire、WirePlumber 或其他程序正在占用该声卡；否则会得到：

```text
Device or resource busy
```

在已经登录的桌面中，不建议为了测试强行停止所有音频服务。可以通过下面的 systemd 服务在开机、音频设备尚未被桌面打开时完成配置。

## 7. 开机和唤醒持久化

仅使用 `alsactl store` 不足以修复 `0x1B`，因为它不能持久化 pin 的 HDA 配置。本文使用：

1. `user_pin_configs` 固定 `0x1B` 为输出；
2. `hda-verb` 在 codec 初始化后再次确认输出和解除静音；
3. `amixer` 恢复 Master、Speaker、Bass Speaker 和麦克风增益；
4. systemd-sleep 钩子在休眠唤醒后重新执行。

### 7.1 修复 helper

创建 `/usr/local/libexec/xingyao-e004-speaker-fix`：

```bash
sudo install -d -m 0755 /usr/local/libexec
sudoedit /usr/local/libexec/xingyao-e004-speaker-fix
```

内容如下。这个版本会按 codec Vendor ID 和 subsystem ID 自动查找设备，不依赖固定的 `card 1`：

```bash
#!/bin/bash
set -euo pipefail
export PATH=/usr/sbin:/usr/bin:/sbin:/bin LC_ALL=C

mode=${1:-boot}
[[ $mode == boot || $mode == resume ]] || exit 2

codec=
for f in /proc/asound/card*/codec#0; do
  if grep -qx 'Vendor Id: 0x10ec0256' "$f" && \
     grep -qx 'Subsystem Id: 0x1d05e004' "$f"; then
    codec=$f
    break
  fi
done

# 不是目标硬件时安静退出，避免影响其他电脑或其他声卡。
[[ -n $codec ]] || exit 0

card=${codec#/proc/asound/card}
card=${card%/codec#0}
id=$(cat "/proc/asound/card${card}/id")
hw=/sys/class/sound/hwC${card}D0
[[ -d $hw ]] || exit 1

headphones=off
if [[ $mode == resume ]]; then
  headphones=$(amixer -c "$id" cget name='Headphone Playback Switch' \
    | sed -n 's/^  : values=//p')
fi

if [[ $mode == boot ]]; then
  desired='0x1b 0x90170120'
  current=$(cat "$hw/user_pin_configs" 2>/dev/null || true)

  # 冷启动时写入用户 pin 配置并重新解析 codec。
  # 已经配置过且当前桌面占用设备时，不重复 reconfig。
  if [[ $current != "$desired" ]] || \
     ! amixer -c "$id" scontrols | grep -q "'Bass Speaker'"; then
    printf '%s\n' "$desired" > "$hw/user_pin_configs"
    echo 1 > "$hw/reconfig"
  fi
fi

# 重新确认第二组扬声器为输出并解除静音。
hda-verb "/dev/snd/hwC${card}D0" 0x1b \
  SET_PIN_WIDGET_CONTROL 0x40 >/dev/null
hda-verb "/dev/snd/hwC${card}D0" 0x1b \
  SET_AMP_GAIN_MUTE 0xb000 >/dev/null

if [[ $mode == boot ]]; then
  amixer -c "$id" sset Master 100% unmute
  amixer -c "$id" sset Speaker 100% unmute
  amixer -c "$id" sset 'Bass Speaker' unmute
  amixer -c "$id" sset Headphone 100% mute
  amixer -c "$id" sset Capture 15dB cap
  amixer -c "$id" sset 'Internal Mic Boost' 20dB
  amixer -c "$id" sset 'Auto-Mute Mode' Disabled
elif [[ $headphones == on,on ]]; then
  # 唤醒时保持耳机模式，不让内置扬声器漏音。
  amixer -c "$id" sset Master 100% unmute
  amixer -c "$id" sset Headphone 100% unmute
  amixer -c "$id" sset Speaker mute
  amixer -c "$id" sset 'Bass Speaker' mute
else
  amixer -c "$id" sset Master 100% unmute
  amixer -c "$id" sset Headphone 100% mute
  amixer -c "$id" sset Speaker 100% unmute
  amixer -c "$id" sset 'Bass Speaker' unmute
fi
```

设置权限：

```bash
sudo chmod 0755 /usr/local/libexec/xingyao-e004-speaker-fix
```

### 7.2 systemd 服务

创建 `/etc/systemd/system/xingyao-e004-speaker-fix.service`：

```ini
[Unit]
Description=Restore XINGYAO ALC256 microphone and speaker routing
After=alsa-restore.service sound.target
Before=display-manager.service
Wants=alsa-restore.service
ConditionPathExists=/proc/asound

[Service]
Type=oneshot
ExecStart=/usr/local/libexec/xingyao-e004-speaker-fix boot
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
```

### 7.3 休眠恢复钩子

创建 `/etc/systemd/system-sleep/xingyao-e004-speaker-fix`：

```bash
#!/bin/bash
case "$1/$2" in
  post/*)
    /usr/local/libexec/xingyao-e004-speaker-fix resume || true
    ;;
esac
```

设置权限并启用：

```bash
sudo chmod 0755 /etc/systemd/system-sleep/xingyao-e004-speaker-fix
sudo systemctl daemon-reload
sudo systemctl enable xingyao-e004-speaker-fix.service
```

**首次安装建议保存工作后自行重启，再检查日志。** 如果尚未完成本次会话的 codec 重新解析，不要直接 `start` 服务；桌面占用声卡时会报 `Device or resource busy`。本次修复是在停止所有占用者、完成临时重新解析后，才成功手动启动服务的。

服务安排在 ALSA 状态恢复之后、显示管理器之前。若发行版启动音频会话的顺序不同，仍可能发生设备占用竞争，需要结合日志调整，不能保证该排序适用于所有 Linux 发行版。没有匹配到目标硬件时 helper 会直接退出，因此服务“成功”不等于扬声器已通过实机验证。

检查：

```bash
systemctl status xingyao-e004-speaker-fix.service
journalctl -b -u xingyao-e004-speaker-fix.service
```

## 8. 3.5mm 耳机切换脚本

本文原本通过脚本直接切换 codec 的扬声器/耳机开关，而不是切换 PipeWire profile。这样可以让播放器保持同一个 sink，避免 Chromium、Edge 等应用因为 sink 被销毁重建而卡住。

脚本路径示例：

```text
~/.local/bin/toggle-audio.sh
```

完整脚本：

```bash
#!/usr/bin/env bash
# 在内置扬声器和 3.5mm 耳机之间切换。
# 不切换 PipeWire profile，避免销毁播放节点。

set -u
CARD=Generic_1

# 固定硬件增益，日常听感音量由 PipeWire 控制。
amixer -c "$CARD" sset Master 100% unmute >/dev/null 2>&1
amixer -c "$CARD" cset name='Speaker Playback Volume' 87 >/dev/null 2>&1
amixer -c "$CARD" cset name='Headphone Playback Volume' 87 >/dev/null 2>&1

hp_state=$(amixer -c "$CARD" cget name='Headphone Playback Switch' 2>/dev/null \
  | awk -F'values=' '/: values=/{print $2}')

if [ "$hp_state" = "on,on" ]; then
  # 耳机 -> 内置扬声器
  amixer -c "$CARD" cset name='Headphone Playback Switch' off,off >/dev/null 2>&1
  amixer -c "$CARD" cset name='Bass Speaker Playback Switch' on,on >/dev/null 2>&1
  amixer -c "$CARD" cset name='Speaker Playback Switch' on,on >/dev/null 2>&1
  notify-send -a "音频切换" "已切换到扬声器" \
    -i audio-speakers 2>/dev/null
else
  # 内置扬声器 -> 耳机
  amixer -c "$CARD" cset name='Bass Speaker Playback Switch' off,off >/dev/null 2>&1
  amixer -c "$CARD" cset name='Speaker Playback Switch' off,off >/dev/null 2>&1
  amixer -c "$CARD" cset name='Headphone Playback Switch' on,on >/dev/null 2>&1
  notify-send -a "音频切换" "已切换到耳机" \
    -i audio-headphones 2>/dev/null
fi
```

设置权限：

```bash
mkdir -p ~/.local/bin
chmod +x ~/.local/bin/toggle-audio.sh
~/.local/bin/toggle-audio.sh
```

可以将 `~/.local/bin/toggle-audio.sh` 绑定到 KDE 的自定义快捷键。

## 9. 验证

### 9.1 查看默认设备和声道

```bash
pactl get-default-sink
pactl get-default-source
pactl list sinks | grep -E 'Name:|Sample Specification:|Channel Map:|Mute:|Volume:'
```

目标输出应包括：

```text
Sample Specification: s32le 2ch 48000Hz
Channel Map: front-left,front-right
```

### 9.2 测试左右扬声器

```bash
aplay -D pipewire /usr/share/sounds/alsa/Front_Left.wav
aplay -D pipewire /usr/share/sounds/alsa/Front_Right.wav
```

### 9.3 测试麦克风

```bash
arecord -D pipewire -f S16_LE -r 48000 -c 2 -d 8 /tmp/mic-test.wav
aplay -D pipewire /tmp/mic-test.wav
```

如果录音有电平但没有人声，应检查默认 source 是否为 HDA 路径，而不是 `acppdmmach`：

```bash
pactl get-default-source
pactl list sources | grep -E 'Name:|Description:|api.alsa.path|Volume:|Mute:'
```

### 9.4 检查两个内置扬声器节点

```bash
sed -n '/^Node 0x14 /,/^Node 0x15 /p' /proc/asound/card1/codec#0
sed -n '/^Node 0x1b /,/^Node 0x1c /p' /proc/asound/card1/codec#0
```

`0x14` 和 `0x1B` 都应为：

```text
Pin-ctls: 0x40: OUT
Amp-Out vals:  [0x00 0x00]
EAPD 0x2: EAPD
```

## 10. 回滚

回滚会恢复到原来的 HiFi/UCM 配置，麦克风和扬声器问题可能重新出现。

停止并删除持久化服务：

```bash
sudo systemctl disable --now xingyao-e004-speaker-fix.service
sudo rm -f /etc/systemd/system/xingyao-e004-speaker-fix.service
sudo rm -f /etc/systemd/system-sleep/xingyao-e004-speaker-fix
sudo rm -f /usr/local/libexec/xingyao-e004-speaker-fix
sudo rm -f /etc/wireplumber/wireplumber.conf.d/51-xingyao-audio.conf
sudo systemctl daemon-reload
```

恢复 WirePlumber：

```bash
systemctl --user restart wireplumber.service
```

清除失效的默认设备：

```bash
wpctl clear-default
```

然后从桌面音频设置中重新选择需要的输入和输出。

如果之前设置过 `user_pin_configs`，该配置只在当前内核运行期间有效，重启后不会写入 BIOS 或固件。要立即恢复固件默认 pin，可以重启电脑；不要把本文的 `0x1B` 修复配置复制到不同 subsystem ID 的设备上。

## 11. 常见问题和注意事项

### 修改后重启 WirePlumber 没有效果

检查配置文件是否放在正确目录：

```bash
ls -l /etc/wireplumber/wireplumber.conf.d/
wireplumber --version
journalctl --user -u wireplumber.service -n 50 --no-pager
```

WirePlumber 0.5 不读取旧版 `main.lua.d` 配置。

### `hda-verb` 报 Permission denied

访问 HDA hwdep 设备通常需要 root 权限：

```bash
sudo hda-verb /dev/snd/hwC1D0 0x1b SET_PIN_WIDGET_CONTROL 0x40
```

不要为了方便而给所有用户放宽 `/dev/snd/hw*` 权限，也不要随意增加 sudoers 或 polkit 规则。

### `reconfig` 报 `Device or resource busy`

说明 PipeWire、WirePlumber 或其他程序仍在使用声卡。可以：

1. 在冷启动服务中完成 `reconfig`；
2. 暂时停止占用声卡的音频服务后再执行；
3. 不要在正在播放视频时强行重新解析 codec。

### 耳机插拔仍不自动切换

本文方案重点是绕过错误的 UCM/ACP 路径并保留稳定播放节点。禁用 ACP 后，系统可能失去原本的自动端口策略，所以使用手动切换脚本更可靠。不同机型需要额外确认耳机检测节点和 mixer 控件。

### 声音仍然小或偏薄

按顺序检查：

```bash
amixer -c Generic_1 sget Master
amixer -c Generic_1 sget Speaker
amixer -c Generic_1 sget 'Bass Speaker'
sed -n '/^Node 0x1b /,/^Node 0x1c /p' /proc/asound/card1/codec#0
```

`Master` 和 `Speaker` 应为 `100% / 0.00dB / on`，`Bass Speaker` 应为 `on`，`0x1B` 应为 `OUT` 且 `Amp-Out vals` 为 `0x00`。

## 12. 结论

这台机器的最终方案不是简单地把声卡切到 `Pro Audio`，而是组合使用：

```text
api.alsa.use-acp = false
        ↓
绕过错误的 ACP-PDM/UCM 麦克风路径

user_pin_configs: 0x1b 0x90170120
        ↓
把第二组内置扬声器固定为输出

hda-verb + amixer
        ↓
解除 0x1B 静音、恢复硬件增益和麦克风增益

PipeWire 双声道规则
        ↓
避免原始 HDA 节点错误使用多声道映射

独立切换脚本
        ↓
在内置扬声器和 3.5mm 耳机之间切换，同时保持播放节点不变
```

该方案适用于本文验证过的 `ALC256 + subsystem 1d05:e004` 硬件。其他机型应先确认 codec 拓扑、mixer 控件、麦克风路径和 HDA pin 定义，再逐项调整，不能把 `0x1B` 命令当作通用 Realtek 修复方法。
