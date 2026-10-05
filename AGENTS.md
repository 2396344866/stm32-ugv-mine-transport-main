# 文档治理规则（对本仓库内所有 Markdown 生效）

## 1 · 一处定义，全文对齐
一个事实只在一个文件里定义。其他文件引用它，不复制数字与标识符。
改定义处时，必须同一批改动里 grep 全库旧值：
    grep -rn "<旧值>" --include=*.md . | grep -v "^./_归档/"
剩余命中必须只剩三类：①「已作废/已归档」说明句 ②既往变更记录 ③ `_归档/` 内留档。
三类之外的每一处都要改。回答里要说得出「还有哪些同类落点是有意不改的」。

## 2 · 一个主题一份主稿
每个主题指定唯一主稿并登记进 `docs/INDEX.md`。分册只承载主稿的展开，
不得复制主稿的结论段。
禁止新建 `xxx_补充.md` / `xxx_v2.md` / `xxx-最终版.md` / `xxx(1).md` / `xxx-副本.md`
这类并行副本；确需并存时，front matter 写明 `status: superseded` 与
`superseded_by: <主稿>`。被取代的稿一律归档，不留在原地。

## 3 · front matter 六字段必填
`status`、`owner`、`last_reviewed`(YYYY-MM-DD)、`scope`、
`supersedes`、`superseded_by`。
`scope` 要写清「不覆盖什么」——缺失等于没有边界。
批量补：`python scripts/sync_md.py fm <root> --owner <你> --apply`

## 4 · 唯一入口
`docs/INDEX.md` 是文档库唯一入口，按主题列主稿 + 路径 + 一句话 scope +
owner + last_reviewed。
新增/归档/合并任何 md，必须同批更新 INDEX。
校验：`validate_md.py <root> --index docs/INDEX.md --strict`
（会检查「磁盘上 status 非 superseded/archived 的 md 是否都在 INDEX 里有行」）

## 5 · 删除 = 归档
不做不可逆删除。移入 `_归档/_deleted_YYYYMMDD_<主题>/`，附 README 写明：
归档清单、原定位、内容去向、现行主稿路径、恢复方式、口径停更提示、
一条未丢清单。归档文件正文一字不改，需要提醒时加横幅。
确需彻底删除，由用户另行指示。

## 6 · 引用随文件走
文件被移动/重命名/归档，必须同批把**全库**引用改到新位置，包括：
Markdown 链接、正文裸路径、索引清单行、主题速查表、合并说明、
以及 `.py` / `.ps1` / 自动化 prompt 里硬编码的文档路径。
改写必须按链接语义重算相对路径，**不得用字符串替换**。
移动/归档后跑 `validate_md.py <root> --report links`，断链必须为 0。

## 7 · 数字与标识符必须回源
「3 层」「55%」「ESP32-S3」这类值，写进任何文档前回源核对：
代码仓库用 `grep` 拿 `file:line`；非 Git 目录用目标文件本体
（逐字读原文 + 自述修订日期 + 同期旁证文件）。
mtime 只是最弱的旁证，复制/解压/云盘同步都会改写它。
证据用 ⚠ 标在句旁：`> 口径来源：<路径> §<节>（<日期>, commit <hash>）`，让人能自己复核。
数量词（「N 层」「N 台」）必须点名核对，数不出来就删数量词、只留名称。
