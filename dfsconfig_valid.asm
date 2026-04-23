_ZN3RRT15dfsconfig_validE4Node:
.LFB7797:
	.loc 33 88 1 is_stmt 1
	.cfi_startproc
	.cfi_personality 0x9b,DW.ref.__gxx_personality_v0
	.cfi_lsda 0x1b,.LLSDA7797
	endbr64
	pushq	%rbp
	.cfi_def_cfa_offset 16
	.cfi_offset 6, -16
	movq	%rsp, %rbp
	.cfi_def_cfa_register 6
	pushq	%r12
	pushq	%rbx
	subq	$320, %rsp
	.cfi_offset 12, -24
	.cfi_offset 3, -32
	movq	%rdi, -328(%rbp)
	movq	%rsi, -336(%rbp)
	.loc 33 88 1
	movq	%fs:40, %rax
	movq	%rax, -24(%rbp)
	xorl	%eax, %eax
	.loc 33 89 32
	movq	-328(%rbp), %rax
	movq	80(%rax), %rax
	.loc 33 89 49
	movq	(%rax), %rax
	movq	(%rax), %r12
	.loc 33 89 32
	movq	-328(%rbp), %rax
	movq	80(%rax), %rbx
	.loc 33 89 89
	movq	-336(%rbp), %rdx
	leaq	-176(%rbp), %rax
	movq	%rdx, %rsi
	movq	%rax, %rdi
.LEHB63:
	call	_ZN4NodeC1ERKS_
.LEHE63:
	.loc 33 89 73 discriminator 2
	movq	-328(%rbp), %rax
	leaq	8(%rax), %rdx
	leaq	-96(%rbp), %rax
	movq	%rdx, %rsi
	movq	%rax, %rdi
.LEHB64:
	call	_ZN7RRTTree18back_parentRRTNodeEv@PLT
.LEHE64:
	.loc 33 89 78 discriminator 4
	leaq	-208(%rbp), %rax
	leaq	-96(%rbp), %rdx
	movq	%rdx, %rsi
	movq	%rax, %rdi
.LEHB65:
	call	_ZN7RRTNode2pcEv@PLT
.LEHE65:
	.loc 33 89 89 discriminator 6
	leaq	-304(%rbp), %rax
	leaq	-176(%rbp), %rcx
	leaq	-208(%rbp), %rdx
	movq	%rbx, %rsi
	movq	%rax, %rdi
.LEHB66:
	call	*%r12
.LVL4:
.LEHE66:
	.loc 33 89 78 discriminator 8
	leaq	-208(%rbp), %rax
	movq	%rax, %rdi
	call	_ZN10PointCloudD1Ev
	.loc 33 89 73 discriminator 1
	leaq	-96(%rbp), %rax
	movq	%rax, %rdi
	call	_ZN7RRTNodeD1Ev
	.loc 33 89 89 discriminator 2
	leaq	-176(%rbp), %rax
	movq	%rax, %rdi
	call	_ZN4NodeD1Ev
.LBB1426:
	.loc 33 91 18
	leaq	-304(%rbp), %rax
	movq	%rax, %rdi
	call	_ZNKSt6vectorI10PointCloudSaIS0_EE4sizeEv
	.loc 33 91 21 discriminator 1
	cmpl	$1, %eax
	sete	%al
	.loc 33 91 2 discriminator 1
	testb	%al, %al
	je	.L492
.LBB1427:
	.loc 33 92 46
	movq	-328(%rbp), %rax
	leaq	8(%rax), %rdx
	leaq	-96(%rbp), %rax
	movq	%rdx, %rsi
	movq	%rax, %rdi
.LEHB67:
	call	_ZN7RRTTree18back_parentRRTNodeEv@PLT
.LEHE67:
	.loc 33 92 52 discriminator 2
	leaq	-272(%rbp), %rax
	leaq	-96(%rbp), %rdx
	movq	%rdx, %rsi
	movq	%rax, %rdi
.LEHB68:
	call	_ZN7RRTNode2pcEv@PLT
.LEHE68:
	.loc 33 92 46 discriminator 4
	leaq	-96(%rbp), %rax
	movq	%rax, %rdi
	call	_ZN7RRTNodeD1Ev
	.loc 33 93 28
	leaq	-304(%rbp), %rax
	movl	$0, %esi
	movq	%rax, %rdi
	call	_ZNSt6vectorI10PointCloudSaIS0_EEixEm
	movq	%rax, %rdx
	.loc 33 93 28 is_stmt 0 discriminator 1
	leaq	-240(%rbp), %rax
	movq	%rdx, %rsi
	movq	%rax, %rdi
.LEHB69:
	call	_ZN10PointCloudC1ERKS_
.LEHE69:
	.loc 33 94 43 is_stmt 1
	leaq	-240(%rbp), %rdx
	leaq	-272(%rbp), %rcx
	movq	-328(%rbp), %rax
	movq	%rcx, %rsi
	movq	%rax, %rdi
	call	_ZN3RRT26calculate_cluster_distanceERK10PointCloudS2_
	movq	%xmm0, %rax
	movq	%rax, -312(%rbp)
	.loc 33 95 14
	movq	-328(%rbp), %rax
	movsd	96(%rax), %xmm1
	.loc 33 95 3
	movsd	-312(%rbp), %xmm0
	comisd	%xmm1, %xmm0
	jbe	.L524
	.loc 33 96 11
	movl	$0, %ebx
	jmp	.L495
.L524:
	.loc 33 98 35
	leaq	-304(%rbp), %rax
	movl	$0, %esi
	movq	%rax, %rdi
	call	_ZNSt6vectorI10PointCloudSaIS0_EEixEm
	movq	%rax, %rdx
	.loc 33 98 36 discriminator 1
	leaq	-96(%rbp), %rax
	movq	%rdx, %rsi
	movq	%rax, %rdi
.LEHB70:
	call	_ZN10PointCloudC1ERKS_
.LEHE70:
	.loc 33 98 36 is_stmt 0 discriminator 2
	movq	-336(%rbp), %rdx
	leaq	-208(%rbp), %rax
	movq	%rdx, %rsi
	movq	%rax, %rdi
.LEHB71:
	call	_ZN4NodeC1ERKS_
.LEHE71:
	.loc 33 98 36 discriminator 4
	leaq	-96(%rbp), %rdx
	leaq	-208(%rbp), %rcx
	leaq	-176(%rbp), %rax
	movq	%rcx, %rsi
	movq	%rax, %rdi
.LEHB72:
	call	_ZN7RRTNodeC1E4Node10PointCloud@PLT
.LEHE72:
	.loc 33 98 36 discriminator 6
	leaq	-208(%rbp), %rax
	movq	%rax, %rdi
	call	_ZN4NodeD1Ev
	.loc 33 98 36 discriminator 1
	leaq	-96(%rbp), %rax
	movq	%rax, %rdi
	call	_ZN10PointCloudD1Ev
	.loc 33 99 15 is_stmt 1
	movq	-328(%rbp), %rax
	leaq	8(%rax), %rbx
	leaq	-176(%rbp), %rdx
	leaq	-96(%rbp), %rax
	movq	%rdx, %rsi
	movq	%rax, %rdi
.LEHB73:
	call	_ZN7RRTNodeC1ERKS_
.LEHE73:
	.loc 33 99 15 is_stmt 0 discriminator 2
	leaq	-96(%rbp), %rax
	movq	%rax, %rsi
	movq	%rbx, %rdi
.LEHB74:
	call	_ZN7RRTTree7replaceE7RRTNode@PLT
.LEHE74:
	.loc 33 99 15 discriminator 4
	leaq	-96(%rbp), %rax
	movq	%rax, %rdi
	call	_ZN7RRTNodeD1Ev
	.loc 33 100 10 is_stmt 1
	movl	$1, %ebx
	.loc 33 101 2
	leaq	-176(%rbp), %rax
	movq	%rax, %rdi
	call	_ZN7RRTNodeD1Ev
.L495:
	.loc 33 101 2 is_stmt 0 discriminator 1
	leaq	-240(%rbp), %rax
	movq	%rax, %rdi
	call	_ZN10PointCloudD1Ev
	.loc 33 101 2 discriminator 2
	leaq	-272(%rbp), %rax
	movq	%rax, %rdi
	call	_ZN10PointCloudD1Ev
	jmp	.L496
.L492:
.LBE1427:
	.loc 33 103 10 is_stmt 1
	movl	$0, %ebx
.L496:
.LBE1426:
	.loc 33 106 1
	leaq	-304(%rbp), %rax
	movq	%rax, %rdi
	call	_ZNSt6vectorI10PointCloudSaIS0_EED1Ev
	movl	%ebx, %eax
	movq	-24(%rbp), %rdx
	subq	%fs:40, %rdx
	je	.L511
	jmp	.L525
.L514:
	endbr64
	.loc 33 89 78 discriminator 7
	movq	%rax, %rbx
	leaq	-208(%rbp), %rax
	movq	%rax, %rdi
	call	_ZN10PointCloudD1Ev
	jmp	.L499
.L513:
	endbr64
	.loc 33 89 73 discriminator 5
	movq	%rax, %rbx
.L499:
	leaq	-96(%rbp), %rax
	movq	%rax, %rdi
	call	_ZN7RRTNodeD1Ev
	jmp	.L500
.L512:
	endbr64
	.loc 33 89 89
	movq	%rax, %rbx
.L500:
	leaq	-176(%rbp), %rax
	movq	%rax, %rdi
	call	_ZN4NodeD1Ev
	movq	%rbx, %rax
	movq	-24(%rbp), %rdx
	subq	%fs:40, %rdx
	je	.L501
	call	__stack_chk_fail@PLT
.L501:
	movq	%rax, %rdi
.LEHB75:
	call	_Unwind_Resume@PLT
.L515:
	endbr64
.LBB1429:
.LBB1428:
	.loc 33 92 46 discriminator 3
	movq	%rax, %rbx
	leaq	-96(%rbp), %rax
	movq	%rax, %rdi
	call	_ZN7RRTNodeD1Ev
	jmp	.L503
.L519:
	endbr64
	.loc 33 98 36 discriminator 5
	movq	%rax, %rbx
	leaq	-208(%rbp), %rax
	movq	%rax, %rdi
	call	_ZN4NodeD1Ev
	jmp	.L505
.L518:
	endbr64
	.loc 33 98 36 is_stmt 0
	movq	%rax, %rbx
.L505:
	leaq	-96(%rbp), %rax
	movq	%rax, %rdi
	call	_ZN10PointCloudD1Ev
	jmp	.L506
.L522:
	endbr64
	.loc 33 99 15 is_stmt 1 discriminator 3
	movq	%rax, %rbx
	leaq	-96(%rbp), %rax
	movq	%rax, %rdi
	call	_ZN7RRTNodeD1Ev
	jmp	.L508
.L521:
	endbr64
	.loc 33 101 2
	movq	%rax, %rbx
.L508:
	leaq	-176(%rbp), %rax
	movq	%rax, %rdi
	call	_ZN7RRTNodeD1Ev
	jmp	.L506
.L520:
	endbr64
	movq	%rax, %rbx
.L506:
	leaq	-240(%rbp), %rax
	movq	%rax, %rdi
	call	_ZN10PointCloudD1Ev
	jmp	.L509
.L517:
	endbr64
	movq	%rax, %rbx
.L509:
	leaq	-272(%rbp), %rax
	movq	%rax, %rdi
	call	_ZN10PointCloudD1Ev
	jmp	.L503
.L516:
	endbr64
.LBE1428:
.LBE1429:
	.loc 33 106 1
	movq	%rax, %rbx
.L503:
	leaq	-304(%rbp), %rax
	movq	%rax, %rdi
	call	_ZNSt6vectorI10PointCloudSaIS0_EED1Ev
	movq	%rbx, %rax
	movq	-24(%rbp), %rdx
	subq	%fs:40, %rdx
	je	.L510
	call	__stack_chk_fail@PLT
.L510:
	movq	%rax, %rdi
	call	_Unwind_Resume@PLT
.LEHE75:
.L525:
	call	__stack_chk_fail@PLT
.L511:
	addq	$320, %rsp
	popq	%rbx
	popq	%r12
	popq	%rbp
	.cfi_def_cfa 7, 8
	ret
	.cfi_endproc
.LFE7797:
	.section	.gcc_except_table
.LLSDA7797:
	.byte	0xff
	.byte	0xff
	.byte	0x1
	.uleb128 .LLSDACSE7797-.LLSDACSB7797
.LLSDACSB7797:
	.uleb128 .LEHB63-.LFB7797
	.uleb128 .LEHE63-.LEHB63
	.uleb128 0
	.uleb128 0
	.uleb128 .LEHB64-.LFB7797
	.uleb128 .LEHE64-.LEHB64
	.uleb128 .L512-.LFB7797
	.uleb128 0
	.uleb128 .LEHB65-.LFB7797
	.uleb128 .LEHE65-.LEHB65
	.uleb128 .L513-.LFB7797
	.uleb128 0
	.uleb128 .LEHB66-.LFB7797
	.uleb128 .LEHE66-.LEHB66
	.uleb128 .L514-.LFB7797
	.uleb128 0
	.uleb128 .LEHB67-.LFB7797
	.uleb128 .LEHE67-.LEHB67
	.uleb128 .L516-.LFB7797
	.uleb128 0
	.uleb128 .LEHB68-.LFB7797
	.uleb128 .LEHE68-.LEHB68
	.uleb128 .L515-.LFB7797
	.uleb128 0
	.uleb128 .LEHB69-.LFB7797
	.uleb128 .LEHE69-.LEHB69
	.uleb128 .L517-.LFB7797
	.uleb128 0
	.uleb128 .LEHB70-.LFB7797
	.uleb128 .LEHE70-.LEHB70
	.uleb128 .L520-.LFB7797
	.uleb128 0
	.uleb128 .LEHB71-.LFB7797
	.uleb128 .LEHE71-.LEHB71
	.uleb128 .L518-.LFB7797
	.uleb128 0
	.uleb128 .LEHB72-.LFB7797
	.uleb128 .LEHE72-.LEHB72
	.uleb128 .L519-.LFB7797
	.uleb128 0
	.uleb128 .LEHB73-.LFB7797
	.uleb128 .LEHE73-.LEHB73
	.uleb128 .L521-.LFB7797
	.uleb128 0
	.uleb128 .LEHB74-.LFB7797
	.uleb128 .LEHE74-.LEHB74
	.uleb128 .L522-.LFB7797
	.uleb128 0
	.uleb128 .LEHB75-.LFB7797
	.uleb128 .LEHE75-.LEHB75
	.uleb128 0
	.uleb128 0
.LLSDACSE7797:
	.text
	.size	_ZN3RRT15dfsconfig_validE4Node, .-_ZN3RRT15dfsconfig_validE4Node
